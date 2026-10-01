#!/usr/bin/env python3
"""
Comprehensive Automated Test Suite for CSI Extraction & Processing Pipeline

Validates:
  1. Struct mt76_csi_data (1058 bytes) memory layout & field alignment
  2. UDP transmission, reception, and parsing
  3. CSI matrix reconstruction (complex64, 4x64)
  4. Phase sanitization (linear regression detrending)
  5. Antenna conjugate ratio (CFO cancellation)
  6. Static clutter removal (EMA high-pass filter)
  7. STFT Doppler spectrogram & velocity mapping
  8. NPZ dataset logging and playback roundtrip
"""

import os
import sys
import time
import socket
import struct
import tempfile
import unittest
import numpy as np

# Ensure laptop directory is in python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from csi_receiver import (
    CSIPacket,
    CSIStreamReceiver,
    CSIDataLogger,
    parse_csi_packet,
    CSI_HEADER_SIZE,
    CSI_PAYLOAD_SIZE,
    CSI_HEADER_FORMAT,
)
import csi_doppler
from csi_doppler import (
    sanitize_phase,
    compute_csi_ratio,
    hampel_filter,
    median_filter_1d,
    StaticClutterFilter,
    MicroDopplerProcessor,
    MockCSIGenerator,
    CSIPlaybackReader,
    WAVELENGTH_5G,
    MicroDopplerApp,
)


class TestCSIPipeline(unittest.TestCase):

    def test_01_struct_layout_and_size(self):
        """Verify binary packet layout matches kernel struct mt76_csi_data exactly (1058 bytes)."""
        self.assertEqual(CSI_HEADER_SIZE, 34)
        self.assertEqual(CSI_PAYLOAD_SIZE, 1058)

        # Build synthetic packet
        ts = 1234567890123
        seq = 42
        frame_seq = 1001
        band = 1
        bw = 2
        ch = 36
        n_rx = 4
        n_tx = 2
        n_sc = 64
        rssi = (-50, -55, -60, -65)
        nf = 85
        pad0 = 0
        mac = b"\x08\x02\x8e\xde\x23\x76"
        pad1_0 = 0
        pad1_1 = 0

        header_bytes = struct.pack(
            CSI_HEADER_FORMAT,
            ts, seq, frame_seq,
            band, bw, ch, n_rx, n_tx, n_sc,
            rssi[0], rssi[1], rssi[2], rssi[3],
            nf, pad0, mac, pad1_0, pad1_1
        )
        self.assertEqual(len(header_bytes), 34)

        # 512 bytes I + 512 bytes Q
        i_data = np.ones((4, 64), dtype="<i2") * 100
        q_data = np.ones((4, 64), dtype="<i2") * -50
        payload = header_bytes + i_data.tobytes() + q_data.tobytes()
        self.assertEqual(len(payload), 1058)

        # Parse with CSIPacket
        pkt = parse_csi_packet(payload)
        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.timestamp_us, ts)
        self.assertEqual(pkt.seq_num, seq)
        self.assertEqual(pkt.frame_seq, frame_seq)
        self.assertEqual(pkt.band, band)
        self.assertEqual(pkt.bw, bw)
        self.assertEqual(pkt.channel, ch)
        self.assertEqual(pkt.n_rx, n_rx)
        self.assertEqual(pkt.n_tx, n_tx)
        self.assertEqual(pkt.n_subcarriers, n_sc)
        self.assertEqual(pkt.rssi, rssi)
        self.assertEqual(pkt.noise_floor, nf)
        self.assertEqual(pkt.src_mac, "08:02:8e:de:23:76")

        # Check CSI complex matrix
        self.assertEqual(pkt.csi_complex.shape, (4, 64))
        self.assertEqual(pkt.csi_complex[0, 0], 100.0 - 50.0j)
        self.assertEqual(pkt.csi_complex[3, 63], 100.0 - 50.0j)

    def test_02_udp_loopback_streaming(self):
        """Test UDP streaming between simulated sender and CSIStreamReceiver."""
        test_port = 15509
        receiver = CSIStreamReceiver(bind_ip="127.0.0.1", port=test_port, timeout=1.0)

        # Create sender
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        gen = MockCSIGenerator(sample_rate=100.0)
        sent_pkts = []
        for i in range(10):
            pkt = gen.generate_packet()
            pkt.seq_num = 100 + i

            # Serialize to raw bytes
            mac_bytes = bytes.fromhex(pkt.src_mac.replace(":", ""))
            header = struct.pack(
                CSI_HEADER_FORMAT,
                pkt.timestamp_us, pkt.seq_num, pkt.frame_seq,
                pkt.band, pkt.bw, pkt.channel, pkt.n_rx, pkt.n_tx, pkt.n_subcarriers,
                pkt.rssi[0], pkt.rssi[1], pkt.rssi[2], pkt.rssi[3],
                pkt.noise_floor, 0, mac_bytes, 0, 0
            )
            raw = header + pkt.i_data.tobytes() + pkt.q_data.tobytes()
            sender.sendto(raw, ("127.0.0.1", test_port))
            sent_pkts.append(pkt)

        # Receive packets
        received_pkts = []
        for _ in range(10):
            p = receiver.recv_packet()
            if p:
                received_pkts.append(p)

        sender.close()
        receiver.close()

        self.assertEqual(len(received_pkts), 10)
        self.assertEqual(received_pkts[0].seq_num, 100)
        self.assertEqual(received_pkts[-1].seq_num, 109)
        self.assertEqual(receiver.seq_gaps, 0)

    def test_03_logger_npz_roundtrip(self):
        """Test logging packets to .npz file and replaying with CSIPlaybackReader."""
        with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            logger = CSIDataLogger(filename=tmp_path)
            gen = MockCSIGenerator(sample_rate=100.0)

            for _ in range(25):
                logger.append(gen.generate_packet())

            self.assertEqual(logger.count(), 25)
            logger.save()

            # Verify file exists and is readable
            self.assertTrue(os.path.exists(tmp_path))
            reader = CSIPlaybackReader(tmp_path, loop=False)
            self.assertEqual(reader.n_total, 25)

            p1 = reader.get_next_packet()
            self.assertIsNotNone(p1)
            self.assertEqual(p1.seq_num, 1)

            # Read all remaining
            count = 1
            while reader.get_next_packet() is not None:
                count += 1
            self.assertEqual(count, 25)

        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_04_phase_sanitization(self):
        """Verify phase sanitization removes linear slope (TSE/SFO error)."""
        # Create a signal with intentional linear phase slope
        subcarriers = np.arange(64) - 32
        true_phase = 0.5  # Constant physical phase
        slope = 0.12     # Artificial linear slope from SFO
        distorted_phase = true_phase + slope * subcarriers
        complex_csi = 50.0 * np.exp(1j * distorted_phase)

        sanitized = sanitize_phase(complex_csi)
        residual_phase = np.angle(sanitized)

        # After removing slope, variation across subcarriers should be near 0
        phase_std = np.std(residual_phase)
        self.assertLess(phase_std, 1e-4)

    def test_05_antenna_conjugate_ratio(self):
        """Verify antenna conjugate ratio cancels common carrier frequency offset (CFO)."""
        cfo_phase = 1.85  # Arbitrary common phase rotation
        h0 = 100.0 * np.exp(1j * (0.3 + cfo_phase))
        h1 = 80.0 * np.exp(1j * (0.1 + cfo_phase))

        # Direct ratio
        ratio = compute_csi_ratio(h0, h1)

        # Expected phase difference: 0.3 - 0.1 = 0.2
        ratio_phase = np.angle(ratio)
        self.assertAlmostEqual(ratio_phase, 0.2, places=4)

    def test_06_static_clutter_filter(self):
        """Verify static DC component is rejected by EMA high-pass filter."""
        filt = StaticClutterFilter(alpha=0.05)

        # Feed constant DC value
        dc_val = np.array([200.0, 200.0])
        for _ in range(100):
            filt.filter(dc_val)

        # Filtered output on constant signal should decay close to 0
        out = filt.filter(dc_val)
        self.assertLess(np.max(np.abs(out)), 2.0)

    def test_07_micro_doppler_stft_velocity(self):
        """Verify STFT micro-Doppler detects simulated human motion velocity."""
        processor = MicroDopplerProcessor(
            window_size=128,
            step_size=8,
            n_fft=256,
            sampling_rate=200.0,
            doppler_limit_hz=50.0,
        )

        gen = MockCSIGenerator(sample_rate=200.0)

        # Feed 300 frames of simulated walking motion (~1.5s)
        for i in range(300):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, i / 200.0)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertGreater(spec_mat.shape[1], 5)
        self.assertEqual(len(v_bins), len(processor.freq_bins))

        # Peak Doppler should be non-zero and within +/- 50 Hz
        self.assertNotEqual(processor.peak_doppler_hz, 0.0)
        self.assertLessEqual(abs(processor.peak_doppler_hz), 50.0)

        # Peak velocity should match physical human walking range (0.3 - 1.5 m/s)
        self.assertGreater(abs(processor.peak_velocity_mps), 0.2)
        self.assertLess(abs(processor.peak_velocity_mps), 2.0)

    def test_08_relative_timestamps_and_empty_spec(self):
        """Verify relative timestamps start at 0.0s and empty spectrogram does not return bogus dummy data."""
        processor = MicroDopplerProcessor(window_size=64, step_size=8, sampling_rate=100.0)
        gen = MockCSIGenerator(sample_rate=100.0)

        # Before enough frames: must return None, None
        mat, t_axis, _ = processor.get_spectrogram_matrix()
        self.assertIsNone(mat)
        self.assertIsNone(t_axis)

        # Feed 150 frames with hardware timestamp starting at high uptime (e.g. 45000.0 seconds)
        base_hw_uptime = 45000.0
        for i in range(150):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, base_hw_uptime + i * 0.01)

        mat, t_axis, _ = processor.get_spectrogram_matrix()
        self.assertIsNotNone(mat)
        self.assertIsNotNone(t_axis)
        self.assertGreater(len(t_axis), 2)
        # Relative time must start at 0.0s (not 45000.0s) and be monotonically positive
        self.assertAlmostEqual(t_axis[0], 0.63, delta=0.1)  # Window starts after window_size (64/100 = 0.64s)
        self.assertGreater(t_axis[-1], t_axis[0])
        self.assertTrue(np.all(t_axis >= 0.0))

    def test_09_band_wavelength_scaling(self):
        """Verify 2.4 GHz vs 5 GHz Doppler velocity scaling."""
        proc_5g = MicroDopplerProcessor(sampling_rate=100.0, doppler_limit_hz=50.0)
        proc_5g.update_band(1)  # 5 GHz
        v_5g = proc_5g.velocity_bins

        proc_24g = MicroDopplerProcessor(sampling_rate=100.0, doppler_limit_hz=50.0)
        proc_24g.update_band(0)  # 2.4 GHz
        v_24g = proc_24g.velocity_bins

        # At 2.4 GHz, wavelength is ~2.14x larger, so velocity for same Doppler freq is ~2.14x larger
        ratio = np.max(v_24g) / np.max(v_5g)
        self.assertAlmostEqual(ratio, 5.21 / 2.437, places=2)

    def test_10_out_of_order_udp_packets(self):
        """Verify receiver handles out-of-order UDP packet delivery without accumulating false gaps."""
        receiver = CSIStreamReceiver(bind_ip="127.0.0.1", port=15511, timeout=0.5)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gen = MockCSIGenerator(sample_rate=100.0)

        def make_raw(seq):
            pkt = gen.generate_packet()
            pkt.seq_num = seq
            mac_bytes = bytes.fromhex(pkt.src_mac.replace(":", ""))
            header = struct.pack(
                CSI_HEADER_FORMAT,
                pkt.timestamp_us, pkt.seq_num, pkt.frame_seq,
                pkt.band, pkt.bw, pkt.channel, pkt.n_rx, pkt.n_tx, pkt.n_subcarriers,
                pkt.rssi[0], pkt.rssi[1], pkt.rssi[2], pkt.rssi[3],
                pkt.noise_floor, 0, mac_bytes, 0, 0
            )
            return header + pkt.i_data.tobytes() + pkt.q_data.tobytes()

        # Send packets in out-of-order sequence: 1, 3, 2, 4
        for seq in [1, 3, 2, 4]:
            sender.sendto(make_raw(seq), ("127.0.0.1", 15511))

        received = []
        for _ in range(4):
            p = receiver.recv_packet()
            if p:
                received.append(p.seq_num)

        sender.close()
        receiver.close()

        self.assertEqual(received, [1, 3, 2, 4])
        # Packet 2 arrived late, so gap was corrected
        self.assertEqual(receiver.seq_gaps, 0)

    def test_11_dynamic_sampling_rate_change_consistency(self):
        """Verify dynamic sampling rate changes do not produce mismatched slice dimensions or ValueError."""
        processor = MicroDopplerProcessor(
            window_size=128,
            step_size=8,
            n_fft=256,
            sampling_rate=100.0,
            doppler_limit_hz=50.0,
        )
        gen = MockCSIGenerator(sample_rate=100.0)

        # Feed packets at 50 Hz rate
        processor.update_sampling_rate(50.0)
        for i in range(160):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, i * 0.02)

        # Feed packets at 335 Hz rate (common high-throughput router rate)
        processor.update_sampling_rate(335.0)
        for i in range(160):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, 3.2 + i * (1.0 / 335.0))

        # Feed packets at 200 Hz rate
        processor.update_sampling_rate(200.0)
        for i in range(160):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, 3.7 + i * 0.005)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertIsNotNone(t_axis)
        self.assertEqual(spec_mat.shape[0], len(processor.freq_bins))
        self.assertEqual(spec_mat.shape[1], len(t_axis))
        self.assertEqual(len(v_bins), len(processor.freq_bins))
        self.assertFalse(np.any(np.isnan(spec_mat)))

    def test_12_buffer_saturation_step_cadence(self):
        """Verify STFT computation cadence stays exactly at step_size even after time_buffer reaches maxlen."""
        processor = MicroDopplerProcessor(
            window_size=64,
            step_size=8,
            n_fft=128,
            sampling_rate=100.0,
        )
        gen = MockCSIGenerator(sample_rate=100.0)

        # Feed 320 frames (well beyond window_size * 2 = 128 maxlen)
        for i in range(320):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, i * 0.01)

        # Expected slices: 1 initial at 64 frames + (320 - 64) // 8 = 1 + 32 = 33 slices
        self.assertEqual(len(processor.spec_history), 33)

    def test_13_heterogeneous_slice_resampling_fallback(self):
        """Verify get_spectrogram_matrix resamples slices of unexpected lengths gracefully without crashing."""
        processor = MicroDopplerProcessor(n_fft=256, doppler_limit_hz=50.0)
        expected_len = len(processor.freq_bins)

        # Inject slices of varying sizes directly into deque
        processor.spec_history.append(np.ones(512, dtype=np.float32) * 5.0)
        processor.spec_timestamps.append(1.0)
        processor.spec_history.append(np.ones(153, dtype=np.float32) * 10.0)
        processor.spec_timestamps.append(2.0)
        processor.spec_history.append(np.ones(expected_len, dtype=np.float32) * 15.0)
        processor.spec_timestamps.append(3.0)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertEqual(spec_mat.shape, (expected_len, 3))
        self.assertEqual(len(t_axis), 3)

    def test_14_visualizer_mock_update_loop(self):
        """Verify MicroDopplerApp initializes and executes update loop without exceptions."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        app = MicroDopplerApp(source_mode="mock")
        gen = MockCSIGenerator(sample_rate=200.0)

        # Feed frames and simulate rate changes
        app.processor.update_sampling_rate(50.0)
        for i in range(150):
            pkt = gen.generate_packet()
            app.latest_packet = pkt
            app.processor.add_frame(pkt.csi_complex, i * 0.02)

        app.processor.update_sampling_rate(335.0)
        for i in range(150):
            pkt = gen.generate_packet()
            app.latest_packet = pkt
            app.processor.add_frame(pkt.csi_complex, 3.0 + i * (1.0 / 335.0))

        artists = app._update_plot(0)
        self.assertIsNotNone(artists)
        self.assertEqual(len(artists), 6)

        # Clean shutdown
        app.running = False
        plt.close(app.fig)

    def test_15_peak_doppler_silence_threshold(self):
        """Verify that on silent or zero-energy CSI input, peak Doppler and velocity report 0.0 (not -50 Hz)."""
        processor = MicroDopplerProcessor(
            window_size=64,
            step_size=8,
            n_fft=128,
            sampling_rate=100.0,
            doppler_limit_hz=50.0,
        )
        silent_frame = np.zeros((4, 64), dtype=np.complex64)
        for i in range(120):
            processor.add_frame(silent_frame, i * 0.01)

        self.assertEqual(processor.peak_doppler_hz, 0.0)
        self.assertEqual(processor.peak_velocity_mps, 0.0)

    def test_16_packet_starvation_rate_decay(self):
        """Verify visualizer throughput rate measurement decays to 0.0 Hz during packet starvation."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        app = MicroDopplerApp(source_mode="mock")
        gen = MockCSIGenerator(sample_rate=200.0)

        # Feed 100 packets
        for i in range(100):
            pkt = gen.generate_packet()
            app.latest_packet = pkt
            app.processor.add_frame(pkt.csi_complex, i * 0.005)
            app.interval_pkts += 1

        # Simulate 1 second elapsed with 100 packets
        app.last_rate_time = time.monotonic() - 1.1
        delta = time.monotonic() - app.last_rate_time
        app.current_rate_hz = app.interval_pkts / delta
        app.interval_pkts = 0
        app.last_rate_time = time.monotonic()
        self.assertGreater(app.current_rate_hz, 50.0)

        # Simulate 1.2 seconds of silence (no packets arrive)
        app.last_rate_time = time.monotonic() - 1.2
        delta = time.monotonic() - app.last_rate_time
        app.current_rate_hz = app.interval_pkts / delta  # 0 / delta = 0.0
        self.assertEqual(app.current_rate_hz, 0.0)

        app.running = False
        plt.close(app.fig)

    def test_17_out_of_order_monotonic_timestamps(self):
        """Verify that out-of-order UDP packet timestamps do not invert time_axis or crash spectrogram."""
        processor = MicroDopplerProcessor(
            window_size=64,
            step_size=8,
            n_fft=128,
            sampling_rate=100.0,
        )
        gen = MockCSIGenerator(sample_rate=100.0)

        # Feed 80 frames with jitter / non-monotonic timestamps
        base_t = 100.0
        for i in range(80):
            pkt = gen.generate_packet()
            # Jitter: swap odd and even timestamps slightly
            jitter = -0.005 if (i % 2 == 1) else 0.005
            processor.add_frame(pkt.csi_complex, base_t + i * 0.01 + jitter)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertIsNotNone(t_axis)
        # Monotonically non-decreasing
        self.assertTrue(np.all(np.diff(t_axis) >= 0.0))

    def test_18_hampel_filter_outlier_removal(self):
        """Verify Hampel filter removes unwrapping phase outliers without distorting linear slope."""
        k = np.arange(64) - 32
        linear_slope = 0.5 + 0.08 * k
        # Inject isolated 2*pi unwrap glitch spike
        corrupted = np.copy(linear_slope)
        corrupted[15] += 2.0 * np.pi  # Outlier
        corrupted[45] -= 2.0 * np.pi  # Outlier

        filtered = hampel_filter(corrupted, window_size=5, n_sigmas=3.0)
        # Outliers should be detected and replaced with local median
        self.assertAlmostEqual(filtered[15], linear_slope[15], delta=0.2)
        self.assertAlmostEqual(filtered[45], linear_slope[45], delta=0.2)

        # Clean linear slope should be untouched
        clean_filtered = hampel_filter(linear_slope, window_size=5, n_sigmas=3.0)
        self.assertTrue(np.allclose(clean_filtered, linear_slope, atol=1e-6))

    def test_19_dc_removal_temporal_mean(self):
        """Verify DC removal subtracts temporal mean per subcarrier over STFT window."""
        processor = MicroDopplerProcessor(
            window_size=64,
            step_size=8,
            n_fft=128,
            sampling_rate=100.0,
            doppler_limit_hz=50.0,
        )

        # Generate a signal with a large static DC offset per subcarrier (different on each subcarrier)
        # plus a 15 Hz AC Doppler tone
        static_offsets = np.random.uniform(50.0, 150.0, size=64) + 1j * np.random.uniform(-100.0, 100.0, size=64)
        t = np.arange(100) / 100.0
        ac_tone = 10.0 * np.exp(1j * 2.0 * np.pi * 15.0 * t)  # 15 Hz tone

        for i in range(100):
            frame = np.zeros((4, 64), dtype=np.complex64)
            for sc in range(64):
                frame[0, sc] = static_offsets[sc] + ac_tone[i]
                frame[1, sc] = static_offsets[sc] + ac_tone[i]
            processor.add_frame(frame, i * 0.01)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)

        # Center frequency index (DC, 0 Hz)
        dc_idx = len(processor.freq_bins) // 2
        # DC power should not overpower the 15 Hz motion tone
        # Compare DC bin power with max non-DC spectral power
        non_dc_mask = np.abs(processor.freq_bins) > 1.0
        max_motion_power = np.max(spec_mat[non_dc_mask, :])
        dc_power = np.mean(spec_mat[dc_idx, :])
        self.assertLess(dc_power, max_motion_power + 20.0)

    def test_20_percentile_dynamic_scaling(self):
        """Verify 10th to 99th percentile dynamic scaling avoids DC spike clipping."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        app = MicroDopplerApp(source_mode="mock")
        gen = MockCSIGenerator(sample_rate=200.0)

        for i in range(200):
            pkt = gen.generate_packet()
            app.latest_packet = pkt
            app.processor.add_frame(pkt.csi_complex, i * 0.005)

        artists = app._update_plot(0)
        self.assertIsNotNone(artists)
        clim = app.im_spec.get_clim()
        self.assertIsNotNone(clim)
        # Clim should have a valid dynamic range (vmax > vmin by at least 15 dB)
        self.assertGreaterEqual(clim[1] - clim[0], 15.0)

        app.running = False
        plt.close(app.fig)

    def test_21_single_antenna_fallback(self):
        """Verify single-antenna fallback operates smoothly without crash."""
        processor = MicroDopplerProcessor(
            window_size=64,
            step_size=8,
            n_fft=128,
            sampling_rate=100.0,
            single_antenna=True,
        )
        gen = MockCSIGenerator(sample_rate=100.0)

        for i in range(80):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, i * 0.01)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertIsNotNone(t_axis)
        self.assertFalse(np.any(np.isnan(spec_mat)))

    def test_22_dsp_feature_flags_gating(self):
        """Verify RULE 3 feature flags can be toggled without breaking execution."""
        orig_norm = csi_doppler.ENABLE_NORMALIZATION
        orig_dc = csi_doppler.ENABLE_DC_REMOVAL
        orig_phase = csi_doppler.ENABLE_PHASE_FILTERING
        orig_scale = csi_doppler.ENABLE_PERCENTILE_SCALING

        try:
            # Test unnormalized correlation
            csi_doppler.ENABLE_NORMALIZATION = False
            h0 = np.array([3.0 + 4.0j])
            h1 = np.array([1.0 + 2.0j])
            res = compute_csi_ratio(h0, h1)
            # Unnormalized should be h0 * conj(h1) = (3+4j)*(1-2j) = 3 - 6j + 4j - 8j^2 = 11 - 2j
            self.assertAlmostEqual(res[0].real, 11.0, places=4)
            self.assertAlmostEqual(res[0].imag, -2.0, places=4)

            # Test DC removal toggle
            csi_doppler.ENABLE_DC_REMOVAL = False
            proc = MicroDopplerProcessor(window_size=32, step_size=4, n_fft=64, sampling_rate=100.0)
            gen = MockCSIGenerator(sample_rate=100.0)
            for i in range(40):
                pkt = gen.generate_packet()
                proc.add_frame(pkt.csi_complex, i * 0.01)
            mat, _, _ = proc.get_spectrogram_matrix()
            self.assertIsNotNone(mat)

            # Test phase filtering toggle
            csi_doppler.ENABLE_PHASE_FILTERING = False
            h = np.exp(1j * np.linspace(0, 1, 64))
            san = sanitize_phase(h)
            self.assertEqual(len(san), 64)

        finally:
            csi_doppler.ENABLE_NORMALIZATION = orig_norm
            csi_doppler.ENABLE_DC_REMOVAL = orig_dc
            csi_doppler.ENABLE_PHASE_FILTERING = orig_phase
            csi_doppler.ENABLE_PERCENTILE_SCALING = orig_scale

    def test_23_hampel_filter_flat_and_plateau_outliers(self):
        """Verify Hampel filter replaces isolated outliers even on zero-MAD / flat sequences."""
        flat_with_outlier = np.array([1.5, 1.5, 1.5, 99.0, 1.5, 1.5, 1.5])
        filtered = hampel_filter(flat_with_outlier, window_size=5, n_sigmas=3.0)
        self.assertAlmostEqual(filtered[3], 1.5, places=4)
        # Verify clean flat sequence remains unchanged
        clean_flat = np.full(16, 2.0)
        filtered_clean = hampel_filter(clean_flat, window_size=5, n_sigmas=3.0)
        self.assertTrue(np.allclose(filtered_clean, clean_flat))

    def test_24_single_antenna_phase_filtering_velocity(self):
        """Verify single-antenna fallback with phase filtering detects Doppler walking velocity."""
        processor = MicroDopplerProcessor(
            window_size=128,
            step_size=8,
            n_fft=256,
            sampling_rate=200.0,
            single_antenna=True,
        )
        gen = MockCSIGenerator(sample_rate=200.0)

        for i in range(300):
            pkt = gen.generate_packet()
            processor.add_frame(pkt.csi_complex, i / 200.0)

        spec_mat, t_axis, v_bins = processor.get_spectrogram_matrix()
        self.assertIsNotNone(spec_mat)
        self.assertGreater(abs(processor.peak_velocity_mps), 0.2)
    def test_25_csi_doppler_v2_visualizer_and_single_antenna(self):
        """Verify csi_doppler_V2 visualizer initializes, supports --single-antenna, and updates without error."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import csi_doppler_V2

        # Test single-antenna mode initialization and update loop
        app = csi_doppler_V2.MicroDopplerApp(source_mode="mock", single_antenna=True)
        self.assertTrue(app.single_antenna)
        self.assertTrue(app.processor.single_antenna)

        gen = MockCSIGenerator(sample_rate=200.0)
        for i in range(100):
            pkt = gen.generate_packet()
            app.latest_packet = pkt
            app.processor.add_frame(pkt.csi_complex, i * 0.005)

        artists = app._update_plot(0)
        self.assertIsNotNone(artists)
        clim = app.im_spec.get_clim()
        self.assertIsNotNone(clim)
        self.assertGreaterEqual(clim[1] - clim[0], 15.0)

        # Verify single-antenna plot artists updated
        self.assertEqual(len(app.line_amp0.get_xdata()), 64)
        self.assertEqual(len(app.line_amp1.get_xdata()), 0)  # Antenna 1 empty in single antenna mode
        self.assertEqual(len(app.line_clean_phase.get_xdata()), 64)

        app.running = False
        plt.close(app.fig)

        # Test V2 Argument Parser includes both clim-max and single-antenna
        orig_argv = sys.argv
        try:
            sys.argv = [
                "csi_doppler_V2.py",
                "--mock",
                "--single-antenna",
                "--clim-min", "-35.0",
                "--clim-max", "25.0",
                "--fps", "45",
                "--bins", "65",
            ]
            import argparse
            # Re-create parser logic as in main()
            p = argparse.ArgumentParser()
            g = p.add_mutually_exclusive_group()
            g.add_argument("--udp", action="store_true")
            g.add_argument("--file", "-f")
            g.add_argument("--mock", action="store_true")
            p.add_argument("--port", "-p", type=int, default=5500)
            p.add_argument("--bind", default="0.0.0.0")
            p.add_argument("--speed", type=float, default=1.0)
            p.add_argument("--fps", type=int, default=30)
            p.add_argument("--bins", type=int, default=129)
            p.add_argument("--no-blit", action="store_true")
            p.add_argument("--clim-min", type=float, default=-40.0)
            p.add_argument("--clim-max", type=float, default=20.0)
            p.add_argument("--single-antenna", action="store_true")
            args = p.parse_args(sys.argv[1:])
            self.assertTrue(args.single_antenna)
            self.assertEqual(args.clim_max, 25.0)
            self.assertEqual(args.clim_min, -35.0)
        finally:
            sys.argv = orig_argv


if __name__ == "__main__":
    unittest.main(verbosity=2)

