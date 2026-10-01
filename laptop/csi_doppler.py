#!/usr/bin/env python3
"""
Netgear R6800 MT7615 Real-Time Micro-Doppler Wi-Fi Sensing Visualizer

Processes Channel State Information (CSI) streams to extract human motion micro-Doppler signatures.
Pipeline steps:
  1. Antenna Conjugate Ratio (H_0 / H_1) to eliminate transceiver CFO & phase noise
  2. Subcarrier Phase Sanitization (linear slope & offset removal)
  3. Static Clutter Removal (high-pass EMA filter, fc ~ 0.5 Hz)
  4. STFT Spectrogram (sliding Hanning window, +/-50 Hz / +/-1.5 m/s velocity)
  5. Multi-panel real-time Matplotlib dashboard

Supports:
  - Live UDP stream from router (--port 5500)
  - Offline playback from recorded .npz file (--file <filename>)
  - Synthesized test signal (--mock) for standalone testing
"""

import sys
import time
import math
import argparse
import threading
from collections import deque
from typing import Optional, Tuple, List, Union

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# Try importing csi_receiver module from same directory
try:
    from laptop.csi_receiver import CSIStreamReceiver, CSIPacket, parse_csi_packet, CSI_PAYLOAD_SIZE
except ImportError:
    try:
        from csi_receiver import CSIStreamReceiver, CSIPacket, parse_csi_packet, CSI_PAYLOAD_SIZE
    except ImportError:
        CSIStreamReceiver = None
        CSIPacket = None
        parse_csi_packet = None
        CSI_PAYLOAD_SIZE = 1058


# Physics constants for Wi-Fi sensing
SPEED_OF_LIGHT = 299792458.0  # m/s
CARRIER_FREQ_24G = 2.437e9     # 2.437 GHz (Channel 6)
WAVELENGTH_24G = SPEED_OF_LIGHT / CARRIER_FREQ_24G  # ~ 0.123 m
CARRIER_FREQ_5G = 5.21e9       # 5.21 GHz (Channel 36-48)
WAVELENGTH_5G = SPEED_OF_LIGHT / CARRIER_FREQ_5G    # ~ 0.0575 m


def sanitize_phase(complex_subcarriers: np.ndarray, remove_offset: bool = False) -> np.ndarray:
    """
    Sanitize phase across subcarriers by removing linear slope (SFO / timing error).
    Input: complex array of shape (N_subcarriers,) or (N_antennas, N_subcarriers)
    Output: sanitized complex array of same shape
    """
    if complex_subcarriers.ndim == 1:
        return _sanitize_phase_1d(complex_subcarriers, remove_offset=remove_offset)
    
    out = np.zeros_like(complex_subcarriers)
    for ant in range(complex_subcarriers.shape[0]):
        out[ant] = _sanitize_phase_1d(complex_subcarriers[ant], remove_offset=remove_offset)
    return out


def _sanitize_phase_1d(h: np.ndarray, remove_offset: bool = False) -> np.ndarray:
    """Sanitize phase for a single antenna 1D array of subcarriers."""
    n_sc = len(h)
    if n_sc == 0:
        return h
    amplitudes = np.abs(h)
    raw_phase = np.angle(h)
    unwrapped = np.unwrap(raw_phase)

    # Subcarrier indices centered around zero (-N/2 to N/2-1)
    k = np.arange(n_sc) - (n_sc // 2)

    # Mask valid subcarriers (avoid zero/null guard carriers)
    valid = amplitudes > (0.01 * np.max(amplitudes) + 1e-6)
    if np.sum(valid) < 4:
        return h

    # Closed-form linear regression: unwrapped(k) = a * k + b
    k_v = k[valid]
    p_v = unwrapped[valid]
    k_mean = np.mean(k_v)
    p_mean = np.mean(p_v)

    denom = np.sum((k_v - k_mean) ** 2)
    if denom == 0:
        return h

    slope = np.sum((k_v - k_mean) * (p_v - p_mean)) / denom

    if remove_offset:
        offset = p_mean - slope * k_mean
        sanitized_phase = unwrapped - (slope * k + offset)
    else:
        # Subtract only linear slope across subcarriers, preserving mean carrier phase
        sanitized_phase = unwrapped - (slope * k)

    return amplitudes * np.exp(1j * sanitized_phase)


def compute_csi_ratio(
    h0: np.ndarray,
    h1: np.ndarray,
    eps: float = 1e-6,
    method: str = "correlation"
) -> np.ndarray:
    """
    Compute antenna conjugate cross-correlation or ratio:
      - 'correlation' (recommended): (h0 * conj(h1)) / (sqrt(|h0|^2 + |h1|^2) + eps)
        Cancels CFO & SFO without division noise explosion at fading notches.
      - 'division': (h0 * conj(h1)) / (|h1|^2 + eps)
    """
    cross = h0 * np.conj(h1)
    if method == "correlation":
        denom = np.sqrt(np.abs(h0) ** 2 + np.abs(h1) ** 2) + eps
        return cross / denom
    else:
        return cross / (np.abs(h1) ** 2 + eps)


class StaticClutterFilter:
    """
    Removes static DC clutter (reflections from walls/furniture)
    using an Exponential Moving Average (EMA) high-pass filter.
    Supports time-aware alpha calculation to handle Wi-Fi packet arrival jitter:
      alpha_i = 1 - exp(-dt / tau), where tau = 1 / (2 * pi * fc).
    """

    def __init__(self, alpha: float = 0.05, fc_hz: float = 0.4):
        self.alpha = float(alpha)
        self.fc_hz = float(fc_hz)
        self.tau = 1.0 / (2.0 * math.pi * self.fc_hz) if self.fc_hz > 0 else 0.4
        self.state: Optional[np.ndarray] = None

    def filter(self, x: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        if self.state is None or self.state.shape != x.shape:
            self.state = np.copy(x)
            return np.zeros_like(x)

        # Dynamic time-aware alpha if dt is provided and valid
        if dt is not None and dt > 0.0001:
            eff_alpha = 1.0 - math.exp(-min(dt, 0.5) / self.tau)
            eff_alpha = max(0.001, min(eff_alpha, 0.5))
        else:
            eff_alpha = self.alpha

        # Update EMA state: state = alpha * x + (1 - alpha) * state
        self.state = eff_alpha * x + (1.0 - eff_alpha) * self.state
        # Dynamic component = input - static baseline
        return x - self.state

    def reset(self):
        self.state = None


class MicroDopplerProcessor:
    """
    Processes time-series of CSI frames into micro-Doppler spectrograms.
    """

    def __init__(
        self,
        window_size: int = 256,
        step_size: int = 16,
        n_fft: int = 512,
        sampling_rate: float = 200.0,
        carrier_freq: float = CARRIER_FREQ_5G,
        doppler_limit_hz: float = 50.0,
        clutter_alpha: float = 0.04,
        num_doppler_bins: Optional[int] = None,
    ):
        self.window_size = int(window_size)
        self.step_size = int(step_size)
        self.n_fft = int(n_fft)
        self.fs = max(float(sampling_rate), 1.0)
        self.carrier_freq = float(carrier_freq)
        self.wavelength = SPEED_OF_LIGHT / self.carrier_freq
        self.doppler_limit_hz = float(doppler_limit_hz)

        self.hanning_win = np.hanning(self.window_size)
        self.clutter_filter = StaticClutterFilter(alpha=clutter_alpha)
        self.t0: Optional[float] = None
        self.frame_count: int = 0
        self.frames_since_last_slice: int = self.step_size
        self._lock = threading.RLock()

        # Ring buffer for raw time series: (buffer_len, 64)
        self.time_buffer = deque(maxlen=self.window_size * 2)
        self.timestamps = deque(maxlen=self.window_size * 2)

        # Spectrogram history buffer (num_time_steps, num_freq_bins)
        self.spectrogram_history_len = 120
        self.spec_history = deque(maxlen=self.spectrogram_history_len)
        self.spec_timestamps = deque(maxlen=self.spectrogram_history_len)

        # Fixed Doppler frequency and velocity grid across [-doppler_limit_hz, +doppler_limit_hz]
        if num_doppler_bins is not None:
            self.num_bins = int(num_doppler_bins)
        else:
            self.num_bins = (self.n_fft // 2 + 1) if (self.n_fft % 2 == 0) else self.n_fft

        self.freq_bins = np.linspace(-self.doppler_limit_hz, self.doppler_limit_hz, self.num_bins)
        self.velocity_bins = self.freq_bins * (self.wavelength / 2.0)

        # Precompute FFT frequency grid based on current sampling rate
        self.current_fft_freqs = np.fft.fftshift(np.fft.fftfreq(self.n_fft, d=1.0 / self.fs))

        # Last peak metrics
        self.peak_doppler_hz = 0.0
        self.peak_velocity_mps = 0.0

    def update_band(self, band: int):
        """Update carrier frequency and velocity bins based on wireless band (0=2.4GHz, 1=5GHz)."""
        with self._lock:
            target_freq = CARRIER_FREQ_5G if band == 1 else CARRIER_FREQ_24G
            if abs(target_freq - self.carrier_freq) > 1e6:
                self.carrier_freq = target_freq
                self.wavelength = SPEED_OF_LIGHT / self.carrier_freq
                self.velocity_bins = self.freq_bins * (self.wavelength / 2.0)

    def update_sampling_rate(self, fs: float):
        """Dynamically update sampling rate if observed packet rate changes."""
        with self._lock:
            if fs > 10.0 and abs(fs - self.fs) > 5.0:
                self.fs = float(fs)
                self.current_fft_freqs = np.fft.fftshift(np.fft.fftfreq(self.n_fft, d=1.0 / self.fs))

    def add_frame(self, csi_frame: np.ndarray, timestamp_s: float, band: Optional[int] = None):
        """
        csi_frame shape: (N_rx, N_sc) complex64, typically (4, 64)
        Computes conjugate ratio between Antenna 0 and Antenna 1, applies phase sanitization.
        """
        if band is not None:
            self.update_band(band)

        with self._lock:
            if self.t0 is None:
                self.t0 = timestamp_s

            rel_t = timestamp_s - self.t0
            # Guard against negative jumps / router reboot
            if rel_t < -1.0:
                self.t0 = timestamp_s
                rel_t = 0.0
            elif self.timestamps and rel_t < self.timestamps[-1]:
                # Preserve monotonic non-decreasing timestamp axis under jitter
                rel_t = self.timestamps[-1]

            dt = (rel_t - self.timestamps[-1]) if self.timestamps else (1.0 / self.fs)

            # Antenna 0 and 1 cross-correlation (cancels CFO & timing SFO, preserves spatial AoA phase slope)
            if csi_frame is not None and np.asarray(csi_frame).ndim >= 2 and csi_frame.shape[0] >= 2:
                h0 = csi_frame[0]
                h1 = csi_frame[1]
                csi_ratio = compute_csi_ratio(h0, h1, method="correlation")
            elif csi_frame is not None and np.asarray(csi_frame).ndim == 1:
                csi_ratio = np.asarray(csi_frame)
            else:
                csi_ratio = np.zeros(64, dtype=np.complex64)

            # Dynamic static clutter filter across time with time-aware alpha
            dynamic_ratio = self.clutter_filter.filter(csi_ratio, dt=dt)

            # Aggregate across active subcarriers (e.g. 4:60 for 20 MHz HT with 56 usable subcarriers)
            if len(dynamic_ratio) > 8:
                sc_start = min(4, len(dynamic_ratio) // 8)
                sc_end = max(sc_start + 1, len(dynamic_ratio) - sc_start)
                agg_val = np.mean(dynamic_ratio[sc_start:sc_end])
            elif len(dynamic_ratio) > 0:
                agg_val = np.mean(dynamic_ratio)
            else:
                agg_val = 0.0 + 0.0j

            if np.isnan(agg_val) or np.isinf(agg_val):
                agg_val = 0.0 + 0.0j

            self.frame_count += 1
            self.frames_since_last_slice += 1
            self.time_buffer.append(agg_val)
            self.timestamps.append(rel_t)

            # If we have enough points, compute STFT slice every step_size frames
            if len(self.time_buffer) >= self.window_size and self.frames_since_last_slice >= self.step_size:
                self._compute_stft_slice(rel_t)
                self.frames_since_last_slice = 0

    def _compute_stft_slice(self, timestamp_s: float):
        """Compute FFT for the latest window and store into spectrogram history."""
        signal_slice = np.array(list(self.time_buffer)[-self.window_size:])
        windowed = signal_slice * self.hanning_win
        fft_vals = np.fft.fftshift(np.fft.fft(windowed, n=self.n_fft))
        power_spectrum = np.abs(fft_vals) ** 2

        # Map current FFT power spectrum to the fixed frequency bins [-doppler_limit_hz, +doppler_limit_hz]
        zoomed_spectrum = np.interp(
            self.freq_bins, self.current_fft_freqs, power_spectrum, left=0.0, right=0.0
        )
        zoomed_spectrum = np.nan_to_num(zoomed_spectrum, nan=0.0, posinf=0.0, neginf=0.0)

        # Log scale (dB) with dynamic range floor
        log_spectrum = 10.0 * np.log10(np.maximum(zoomed_spectrum, 1e-12))
        log_spectrum = np.nan_to_num(log_spectrum, nan=-120.0)

        # Peak Doppler detection (excluding DC +/- 1 Hz, requiring non-negligible spectral energy)
        non_dc = np.abs(self.freq_bins) > 1.0
        if np.any(non_dc) and np.max(zoomed_spectrum[non_dc]) > 1e-9:
            peak_idx = np.argmax(zoomed_spectrum[non_dc])
            self.peak_doppler_hz = float(self.freq_bins[non_dc][peak_idx])
            self.peak_velocity_mps = float(self.velocity_bins[non_dc][peak_idx])
        else:
            self.peak_doppler_hz = 0.0
            self.peak_velocity_mps = 0.0

        self.spec_history.append(log_spectrum)
        self.spec_timestamps.append(timestamp_s)

    def get_spectrogram_matrix(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
        """
        Returns (spec_matrix, time_axis, velocity_axis)
        spec_matrix shape: (len(freq_bins), num_time_steps)
        Returns (None, None, velocity_axis) if fewer than 2 slices are available.
        """
        with self._lock:
            if len(self.spec_history) < 2:
                return None, None, self.velocity_bins

            target_len = len(self.freq_bins)
            aligned_slices = []
            aligned_timestamps = []

            for s, t in zip(list(self.spec_history), list(self.spec_timestamps)):
                s_arr = np.asarray(s).ravel()
                if len(s_arr) == target_len:
                    aligned_slices.append(s_arr)
                    aligned_timestamps.append(t)
                elif len(s_arr) > 0:
                    x_old = np.linspace(-self.doppler_limit_hz, self.doppler_limit_hz, len(s_arr))
                    resampled = np.interp(self.freq_bins, x_old, s_arr)
                    aligned_slices.append(resampled)
                    aligned_timestamps.append(t)

            if len(aligned_slices) < 2:
                return None, None, self.velocity_bins

            matrix = np.column_stack(aligned_slices)  # (F, T)
            t_axis = np.array(aligned_timestamps)
            return matrix, t_axis, self.velocity_bins


class MockCSIGenerator:
    """Synthesizes realistic CSI stream with simulated human walking motion for testing."""

    def __init__(self, sample_rate: float = 200.0):
        self.fs = sample_rate
        self.t = 0.0
        self.seq = 0
        self.cfo_drift = 0.0
        self.motion_phase = 0.0

    def generate_packet(self) -> CSIPacket:
        dt = 1.0 / self.fs
        self.t += dt
        self.seq += 1

        # Simulate CFO drift common to all antennas
        self.cfo_drift += 2.0 * math.pi * 35.0 * dt
        common_phase_noise = np.exp(1j * self.cfo_drift)

        # Simulate human motion Doppler:
        # A walking person creates a time-varying Doppler signature:
        # e.g., walking towards router at 1.0 m/s -> ~35 Hz Doppler
        # arm swing oscillation at 1.8 Hz modulates +/- 15 Hz
        walking_speed = 0.8 + 0.3 * math.sin(2.0 * math.pi * 0.25 * self.t)  # ~0.5 to 1.1 m/s
        arm_swing = 0.4 * math.sin(2.0 * math.pi * 1.8 * self.t)
        total_velocity = walking_speed + arm_swing
        doppler_freq = (2.0 * total_velocity / WAVELENGTH_5G)
        self.motion_phase += 2.0 * math.pi * doppler_freq * dt

        # Static multipath + dynamic Doppler path
        static_multipath = 150.0 * np.exp(1j * 0.5)
        dynamic_multipath = 40.0 * np.exp(1j * self.motion_phase)

        subcarrier_k = np.arange(64) - 32
        sfo_slope = 0.05 * math.sin(0.1 * self.t)  # Sampling frequency offset

        csi_complex = np.zeros((4, 64), dtype=np.complex64)
        for ant in range(4):
            ant_phase_offset = ant * (math.pi / 3.0)
            phase_profile = ant_phase_offset + sfo_slope * subcarrier_k
            noise = (np.random.randn(64) + 1j * np.random.randn(64)) * 3.0

            # Signal on antenna: common CFO * (static + dynamic*ant_weight) + noise
            ant_weight = math.exp(-0.4 * ant)
            h = common_phase_noise * np.exp(1j * phase_profile) * (
                static_multipath + dynamic_multipath * ant_weight
            ) + noise
            csi_complex[ant] = h.astype(np.complex64)

        i_data = np.real(csi_complex).astype(np.int16)
        q_data = np.imag(csi_complex).astype(np.int16)

        return CSIPacket(
            timestamp_us=int(self.t * 1e6),
            seq_num=self.seq,
            frame_seq=self.seq % 4096,
            band=1,  # 5 GHz
            bw=2,    # 80 MHz / 20 MHz
            channel=36,
            n_rx=4,
            n_tx=2,
            n_subcarriers=64,
            rssi=(-52, -55, -58, -60),
            noise_floor=83,
            src_mac="08:02:8e:de:23:76",
            i_data=i_data,
            q_data=q_data,
            csi_complex=csi_complex,
        )


class CSIPlaybackReader:
    """Reads recorded .npz file and replays CSI frames at specified speed."""

    def __init__(self, filename: str, speed: float = 1.0, loop: bool = True):
        self.filename = filename
        self.speed = speed
        self.loop = loop

        print(f"[+] Loading recorded CSI dataset: {filename}...")
        data = np.load(filename)
        self.timestamps_us = data["timestamp_us"]
        self.seq_nums = data["seq_num"]
        self.frame_seqs = data["frame_seq"]
        self.band = data["band"]
        self.bw = data["bw"]
        self.channel = data["channel"]
        self.n_rx = data["n_rx"]
        self.n_tx = data["n_tx"]
        self.n_subcarriers = data["n_subcarriers"]
        self.rssi = data["rssi"]
        self.noise_floor = data["noise_floor"]
        self.src_macs = data["src_mac"]
        self.csi = data["csi"]
        self.i_data = data["i_data"]
        self.q_data = data["q_data"]

        self.n_total = len(self.timestamps_us)
        self.current_idx = 0
        print(f"[OK] Loaded {self.n_total} records.")

    def get_next_packet(self) -> Optional[CSIPacket]:
        if self.current_idx >= self.n_total:
            if self.loop:
                self.current_idx = 0
            else:
                return None

        idx = self.current_idx
        self.current_idx += 1

        mac_val = self.src_macs[idx]
        mac_str = str(mac_val) if isinstance(mac_val, (str, np.str_)) else "00:00:00:00:00:00"

        return CSIPacket(
            timestamp_us=int(self.timestamps_us[idx]),
            seq_num=int(self.seq_nums[idx]),
            frame_seq=int(self.frame_seqs[idx]),
            band=int(self.band[idx]),
            bw=int(self.bw[idx]),
            channel=int(self.channel[idx]),
            n_rx=int(self.n_rx[idx]),
            n_tx=int(self.n_tx[idx]),
            n_subcarriers=int(self.n_subcarriers[idx]),
            rssi=tuple(int(r) for r in self.rssi[idx]),
            noise_floor=int(self.noise_floor[idx]),
            src_mac=mac_str,
            i_data=self.i_data[idx],
            q_data=self.q_data[idx],
            csi_complex=self.csi[idx],
        )


class MicroDopplerApp:
    """Matplotlib Real-Time Micro-Doppler GUI Application."""

    def __init__(
        self,
        source_mode: str = "udp",
        port: int = 5500,
        bind_ip: str = "0.0.0.0",
        file_path: Optional[str] = None,
        speed: float = 1.0,
    ):
        self.source_mode = source_mode
        self.port = port
        self.bind_ip = bind_ip
        self.file_path = file_path
        self.speed = speed

        self.running = True
        self.processor = MicroDopplerProcessor()

        # Thread-safe buffer for incoming packets
        self.packet_queue = deque(maxlen=2048)
        self.latest_packet: Optional[CSIPacket] = None

        # Statistics
        self.pkt_count = 0
        self.start_time = time.monotonic()
        self.last_rate_time = self.start_time
        self.interval_pkts = 0
        self.current_rate_hz = 0.0

        # Start acquisition background thread
        self.worker_thread = threading.Thread(target=self._acquisition_worker, daemon=True)
        self.worker_thread.start()

        # Build Matplotlib UI
        self._setup_figure()

    def _acquisition_worker(self):
        """Background thread acquiring CSI packets from UDP, mock, or file."""
        receiver = None
        mock_gen = None
        file_reader = None

        if self.source_mode == "udp":
            if CSIStreamReceiver is None:
                print("Error: CSIStreamReceiver not available.")
                return
            receiver = CSIStreamReceiver(bind_ip=self.bind_ip, port=self.port, timeout=0.2)
        elif self.source_mode == "mock":
            mock_gen = MockCSIGenerator(sample_rate=200.0)
        elif self.source_mode == "file":
            file_reader = CSIPlaybackReader(self.file_path, speed=self.speed)

        prev_time = time.monotonic()

        while self.running:
            pkt = None
            if self.source_mode == "udp" and receiver:
                pkt = receiver.recv_packet()
            elif self.source_mode == "mock" and mock_gen:
                time.sleep(1.0 / 200.0)
                pkt = mock_gen.generate_packet()
            elif self.source_mode == "file" and file_reader:
                time.sleep(0.005 / self.speed)
                pkt = file_reader.get_next_packet()

            if pkt is not None:
                # Use packet hardware timestamp if valid, otherwise monotonic
                pkt_t_s = (pkt.timestamp_us / 1e6) if pkt.timestamp_us > 0 else time.monotonic()
                self.processor.add_frame(pkt.csi_complex, pkt_t_s, band=pkt.band)
                self.latest_packet = pkt
                self.pkt_count += 1
                self.interval_pkts += 1

            now = time.monotonic()
            delta = now - self.last_rate_time
            if delta >= 1.0:
                self.current_rate_hz = self.interval_pkts / delta
                if self.interval_pkts > 0:
                    self.processor.update_sampling_rate(self.current_rate_hz)
                self.interval_pkts = 0
                self.last_rate_time = now

        if receiver:
            receiver.close()

    def _setup_figure(self):
        """Construct multi-panel matplotlib dashboard."""
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(13, 8), dpi=100)
        self.fig.canvas.manager.set_window_title(
            f"Netgear R6800 MT7615 Micro-Doppler Sensing - [{self.source_mode.upper()}]"
        )

        gs = GridSpec(2, 2, figure=self.fig, height_ratios=[1.0, 1.4], hspace=0.32, wspace=0.25)

        # Panel 1: Subcarrier Amplitudes
        self.ax_amp = self.fig.add_subplot(gs[0, 0])
        self.ax_amp.set_title("Subcarrier Amplitudes (Antenna 0 & 1)", fontsize=11, fontweight="bold", pad=8)
        self.ax_amp.set_xlabel("Subcarrier Index (0 - 63)", fontsize=9)
        self.ax_amp.set_ylabel("Amplitude |H|", fontsize=9)
        self.ax_amp.set_xlim(0, 63)
        self.ax_amp.set_ylim(0, 400)
        self.ax_amp.grid(True, linestyle="--", alpha=0.3)
        (self.line_amp0,) = self.ax_amp.plot([], [], label="Ant 0 (Rx0)", color="#00ffcc", lw=1.8)
        (self.line_amp1,) = self.ax_amp.plot([], [], label="Ant 1 (Rx1)", color="#ff007f", lw=1.8)
        self.ax_amp.legend(loc="upper right", fontsize=8)

        # Panel 2: Phase Sanitization & CSI Ratio
        self.ax_phase = self.fig.add_subplot(gs[0, 1])
        self.ax_phase.set_title("CSI Conjugate Ratio Phase (Sanitized)", fontsize=11, fontweight="bold", pad=8)
        self.ax_phase.set_xlabel("Subcarrier Index (0 - 63)", fontsize=9)
        self.ax_phase.set_ylabel("Phase (radians)", fontsize=9)
        self.ax_phase.set_xlim(0, 63)
        self.ax_phase.set_ylim(-math.pi * 1.5, math.pi * 1.5)
        self.ax_phase.grid(True, linestyle="--", alpha=0.3)
        (self.line_raw_phase,) = self.ax_phase.plot([], [], label="Raw Ratio Phase", color="#888888", lw=1.2, ls=":")
        (self.line_clean_phase,) = self.ax_phase.plot([], [], label="Sanitized Ratio Phase", color="#ffff00", lw=2.0)
        self.ax_phase.legend(loc="upper right", fontsize=8)

        # Panel 3: Micro-Doppler Spectrogram (spans full width bottom)
        self.ax_spec = self.fig.add_subplot(gs[1, :])
        self.ax_spec.set_title(
            "Live Micro-Doppler Spectrogram: Time vs Doppler Shift / Target Velocity",
            fontsize=12,
            fontweight="bold",
            pad=8,
        )
        self.ax_spec.set_xlabel("Time (seconds)", fontsize=10)
        self.ax_spec.set_ylabel("Doppler Shift (Hz)", fontsize=10)

        # Secondary Y axis for Target Velocity (m/s)
        self.ax_vel = self.ax_spec.twinx()
        self.ax_vel.set_ylabel("Target Velocity (m/s)", fontsize=10, color="#ff9900")
        self.ax_vel.tick_params(axis="y", labelcolor="#ff9900")

        d_lim = self.processor.doppler_limit_hz
        v_max = d_lim * (self.processor.wavelength / 2.0)
        self.ax_spec.set_ylim(-d_lim, d_lim)
        self.ax_vel.set_ylim(-v_max, v_max)

        # Initial dummy spectrogram image
        dummy_data = np.zeros((len(self.processor.freq_bins), 100))
        self.im_spec = self.ax_spec.imshow(
            dummy_data,
            aspect="auto",
            origin="lower",
            cmap="inferno",
            extent=[0, 10, -d_lim, d_lim],
            vmin=-20,
            vmax=40,
            interpolation="bilinear",
        )
        self.cbar = self.fig.colorbar(self.im_spec, ax=[self.ax_spec, self.ax_vel], pad=0.08, fraction=0.03)
        self.cbar.set_label("Spectral Power (dB)", fontsize=9)

        # Text banner for real-time status overlay
        self.status_text = self.fig.text(
            0.5,
            0.02,
            "Rate: 0.0 Hz | Total: 0 pkts | Velocity: 0.00 m/s | Doppler: 0.0 Hz",
            ha="center",
            fontsize=10,
            color="#33ff33",
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", fc="#111111", ec="#33ff33", lw=1.0),
        )

    def _update_plot(self, frame_idx):
        """Matplotlib animation update step."""
        # Update Panel 1 & 2 if we have a recent packet
        if self.latest_packet is not None:
            pkt = self.latest_packet
            if pkt.csi_complex is not None and np.asarray(pkt.csi_complex).ndim >= 2 and pkt.csi_complex.shape[0] >= 2:
                h0 = pkt.csi_complex[0]
                h1 = pkt.csi_complex[1]

                # Amplitudes
                amp0 = np.abs(h0)
                amp1 = np.abs(h1)
                x_sc = np.arange(len(amp0))

                self.line_amp0.set_data(x_sc, amp0)
                self.line_amp1.set_data(x_sc, amp1)
                if len(amp0) > 0 and len(amp1) > 0:
                    max_amp = max(float(np.max(amp0)), float(np.max(amp1)), 10.0)
                    self.ax_amp.set_ylim(0, max_amp * 1.25)

                # Ratio & Sanitized Phase
                csi_ratio = compute_csi_ratio(h0, h1)
                raw_ratio_phase = np.angle(csi_ratio)
                sanitized_ratio = _sanitize_phase_1d(csi_ratio)
                clean_phase = np.angle(sanitized_ratio)

                self.line_raw_phase.set_data(x_sc, raw_ratio_phase)
                self.line_clean_phase.set_data(x_sc, clean_phase)

        # Update Panel 3: Spectrogram
        spec_mat, t_axis, v_bins = self.processor.get_spectrogram_matrix()
        if spec_mat is not None and len(t_axis) >= 2:
            t_min = float(t_axis[0])
            t_max = float(t_axis[-1])
            if t_max <= t_min:
                t_max = t_min + 0.1

            d_lim = self.processor.doppler_limit_hz
            self.im_spec.set_data(spec_mat)
            self.im_spec.set_extent([t_min, t_max, -d_lim, d_lim])
            self.ax_spec.set_xlim(t_min, t_max)
            self.ax_spec.set_ylim(-d_lim, d_lim)
            if v_bins is not None and len(v_bins) >= 2:
                self.ax_vel.set_ylim(float(v_bins[0]), float(v_bins[-1]))

            # Dynamic color scaling based on finite spectral values
            finite_vals = spec_mat[np.isfinite(spec_mat)]
            if len(finite_vals) > 0:
                p98 = float(np.percentile(finite_vals, 98))
                p15 = float(np.percentile(finite_vals, 15))
                self.im_spec.set_clim(vmin=p15, vmax=max(p98, p15 + 15.0))

        # Status text
        vel = self.processor.peak_velocity_mps
        dop = self.processor.peak_doppler_hz
        rssi_str = ""
        if self.latest_packet:
            rssi_str = f" | RSSI: {self.latest_packet.rssi[:2]}"

        status = (
            f"Rate: {self.current_rate_hz:5.1f} Hz | Total: {self.pkt_count:6d} pkts"
            f"{rssi_str} | Peak Doppler: {dop:+5.1f} Hz | Est Velocity: {vel:+5.2f} m/s"
        )
        self.status_text.set_text(status)

        return (
            self.line_amp0,
            self.line_amp1,
            self.line_raw_phase,
            self.line_clean_phase,
            self.im_spec,
            self.status_text,
        )

    def run(self):
        """Launch the live visualization loop."""
        anim = animation.FuncAnimation(
            self.fig,
            self._update_plot,
            interval=40,  # ~25 FPS UI refresh
            blit=False,
            cache_frame_data=False,
        )
        try:
            plt.show()
        finally:
            self.running = False
            if self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(
        description="Netgear R6800 MT7615 Micro-Doppler Sensing Visualizer"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--udp", action="store_true", help="Stream live from UDP (default)")
    group.add_argument("--file", "-f", help="Replay recorded CSI dataset (.npz file)")
    group.add_argument("--mock", action="store_true", help="Run with simulated walking motion (no router needed)")

    parser.add_argument("--port", "-p", type=int, default=5500, help="UDP port (default: 5500)")
    parser.add_argument("--bind", default="0.0.0.0", help="UDP bind IP (default: 0.0.0.0)")
    parser.add_argument("--speed", type=float, default=1.0, help="File playback speed multiplier (default: 1.0)")

    args = parser.parse_args()

    mode = "mock" if args.mock else ("file" if args.file else "udp")

    print("=" * 68)
    print("  Netgear R6800 MT7615 Micro-Doppler Visualizer Active")
    print("=" * 68)
    print(f"  Mode:           {mode.upper()}")
    if mode == "udp":
        print(f"  UDP Source:     {args.bind}:{args.port}")
    elif mode == "file":
        print(f"  Playback File:  {args.file} (speed: {args.speed}x)")
    elif mode == "mock":
        print("  Simulation:     Human walking & arm swing (+/- 1.4 m/s Doppler)")
    print("  Close plot window to exit.")
    print("=" * 68 + "\n")

    app = MicroDopplerApp(
        source_mode=mode,
        port=args.port,
        bind_ip=args.bind,
        file_path=args.file,
        speed=args.speed,
    )
    app.run()


if __name__ == "__main__":
    main()
