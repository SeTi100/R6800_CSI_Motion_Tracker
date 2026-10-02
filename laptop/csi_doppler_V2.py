#!/usr/bin/env python3
"""
Netgear R6800 MT7615 Real-Time Micro-Doppler Wi-Fi Sensing Visualizer - V2 High-Performance Edition

Optimized for ultra-low latency, fluid 30-60 FPS UI rendering, and smooth Windows window dragging.
Pipeline steps:
  1. Antenna Conjugate Ratio (H_0 / H_1) to eliminate transceiver CFO & phase noise
  2. Subcarrier Phase Sanitization (linear slope & offset removal)
  3. Static Clutter Removal (high-pass EMA filter, fc ~ 0.5 Hz)
  4. STFT Spectrogram (sliding Hanning window, +/-50 Hz / +/-1.5 m/s velocity)
  5. Multi-panel real-time Matplotlib dashboard with hardware-efficient blitting

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
import matplotlib
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

# DSP Feature Flags (RULE 3 compliance)
ENABLE_NORMALIZATION = True          # Amplitude normalization in compute_csi_ratio
ENABLE_DC_REMOVAL = True             # Subtract temporal mean per subcarrier over STFT window
ENABLE_PHASE_FILTERING = True        # Hampel / median filtering on raw phases to eliminate unwrap outliers
PHASE_FILTER_METHOD = "hampel"       # "hampel" or "median"
ENABLE_PERCENTILE_SCALING = True     # 10th-99th percentile dynamic scaling for spectrogram clim
ENABLE_SINGLE_ANTENNA = False        # Fallback to single antenna instead of antenna conjugate cross-correlation
ENABLE_PCA = True                    # Principal Component Analysis to extract PC1 (>80% dynamic variance)
ENABLE_SUBCARRIER_PREFILTER = False  # Pre-filter subcarriers with Hampel/Median filter (disabled by default for performance)
BISTATIC_ANGLE_DEG_DEFAULT = 0.0     # Bistatic angle beta in degrees (default 0.0)
TARGET_HEADING_DEG_DEFAULT = 0.0     # Target motion heading theta in degrees (default 0.0)


def bistatic_velocity_factor(
    wavelength: float,
    bistatic_angle_rad: float = 0.0,
    target_heading_rad: float = 0.0,
) -> float:
    """
    Computes velocity conversion factor K such that v = f_D * K using bistatic Doppler equation:
      f_D = (2 * v / lambda) * cos(theta) * cos(beta / 2)
      => v = f_D * lambda / (2 * cos(theta) * cos(beta / 2))
    where beta is the bistatic angle and theta is the target heading relative to bistatic bisector.
    """
    cos_geom = math.cos(target_heading_rad) * math.cos(bistatic_angle_rad / 2.0)
    if abs(cos_geom) < 1e-4:
        cos_geom = 1e-4 if cos_geom >= 0 else -1e-4
    return wavelength / (2.0 * cos_geom)


def prefilter_subcarriers(
    x: np.ndarray,
    method: str = "hampel",
    window_size: int = 5,
    n_sigmas: float = 3.0,
) -> np.ndarray:
    """
    Pre-filter subcarrier stream across tones to remove outliers/impulse noise.
    Supports Hampel filter or Median filter.
    """
    arr = np.asarray(x)
    if arr.size == 0:
        return np.copy(arr)

    if arr.ndim == 1:
        if np.iscomplexobj(arr):
            if method == "median":
                re = median_filter_1d(arr.real, window_size=window_size)
                im = median_filter_1d(arr.imag, window_size=window_size)
            else:
                re = hampel_filter(arr.real, window_size=window_size, n_sigmas=n_sigmas)
                im = hampel_filter(arr.imag, window_size=window_size, n_sigmas=n_sigmas)
            return re + 1j * im
        else:
            if method == "median":
                return median_filter_1d(arr, window_size=window_size)
            return hampel_filter(arr, window_size=window_size, n_sigmas=n_sigmas)

    out = np.zeros_like(arr)
    for i in range(arr.shape[0]):
        out[i] = prefilter_subcarriers(arr[i], method=method, window_size=window_size, n_sigmas=n_sigmas)
    return out


def extract_pca_component(
    subcarriers: np.ndarray,
    min_explained_variance_ratio: float = 0.5,
) -> Tuple[np.ndarray, float]:
    """
    Extract the first Principal Component (PC1) across subcarriers.
    Input: subcarriers of shape (N_time, N_subcarriers) complex.
    Returns: (pc1_signal, explained_variance_ratio)
      pc1_signal: 1D complex array of length N_time representing dominant motion signature.
      explained_variance_ratio: fraction of dynamic variance explained by PC1 (0.0 to 1.0).
    """
    if subcarriers.ndim != 2 or subcarriers.shape[0] < 2 or subcarriers.shape[1] < 1:
        return (np.mean(subcarriers, axis=-1) if subcarriers.ndim >= 2 else subcarriers), 0.0

    x_c = subcarriers - np.mean(subcarriers, axis=0, keepdims=True)
    try:
        u, s, vh = np.linalg.svd(x_c, full_matrices=False)
        total_var = float(np.sum(s ** 2))
        if total_var > 1e-12:
            evr = float((s[0] ** 2) / total_var)
            pc1 = u[:, 0] * s[0]
            return pc1, evr
        else:
            return np.mean(subcarriers, axis=1), 0.0
    except Exception:
        return np.mean(subcarriers, axis=1), 0.0


def hampel_filter(x: np.ndarray, window_size: int = 5, n_sigmas: float = 3.0) -> np.ndarray:
    """
    Fast vectorized Hampel filter to detect and replace outliers in a 1D sequence.
    Computes rolling median and Median Absolute Deviation (MAD).
    Points deviating by more than n_sigmas * MAD from the median are replaced with the median.
    """
    x_arr = np.asarray(x, dtype=float)
    n = len(x_arr)
    if n < window_size:
        return np.copy(x_arr)

    k = window_size // 2
    padded = np.pad(x_arr, k, mode='edge')
    windows = np.lib.stride_tricks.sliding_window_view(padded, window_size)
    med = np.median(windows, axis=-1)
    mad = 1.4826 * np.median(np.abs(windows - med[:, None]), axis=-1)
    thresh = np.where(mad > 1e-6, n_sigmas * mad, 1e-3)
    out = np.copy(x_arr)
    mask = np.abs(x_arr - med) > thresh
    out[mask] = med[mask]
    return out


def median_filter_1d(x: np.ndarray, window_size: int = 3) -> np.ndarray:
    """Standard 1D median filter."""
    x_arr = np.asarray(x, dtype=float)
    n = len(x_arr)
    if n < window_size:
        return np.copy(x_arr)
    y = np.copy(x_arr)
    k = window_size // 2
    for i in range(n):
        start = max(0, i - k)
        end = min(n, i + k + 1)
        y[i] = float(np.median(x_arr[start:end]))
    return y


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

    # Set Hampel or median filtering to eliminate unwrap outliers (RULE 3 gated)
    if ENABLE_PHASE_FILTERING:
        if PHASE_FILTER_METHOD == "median":
            unwrapped = median_filter_1d(unwrapped, window_size=3)
        else:
            unwrapped = hampel_filter(unwrapped, window_size=5, n_sigmas=3.0)

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
        if ENABLE_NORMALIZATION:
            denom = np.sqrt(np.abs(h0) ** 2 + np.abs(h1) ** 2) + eps
            return cross / denom
        return cross
    else:
        return cross / (np.abs(h1) ** 2 + eps)


class StaticClutterFilter:
    """
    Removes static DC clutter (reflections from walls/furniture)
    using an Exponential Moving Average (EMA) high-pass filter.
    Equivalent to a single-pole IIR high-pass filter with cutoff fc ~ 0.5 Hz.
    """

    def __init__(self, alpha: float = 0.05):
        self.alpha = alpha
        self.state: Optional[np.ndarray] = None

    def filter(self, x: np.ndarray) -> np.ndarray:
        if self.state is None or self.state.shape != x.shape:
            self.state = np.copy(x)
            return np.zeros_like(x)

        # Update EMA state: state = alpha * x + (1 - alpha) * state
        self.state = self.alpha * x + (1.0 - self.alpha) * self.state
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
        single_antenna: bool = False,
        bistatic_angle_deg: float = BISTATIC_ANGLE_DEG_DEFAULT,
        target_heading_deg: float = TARGET_HEADING_DEG_DEFAULT,
        use_pca: bool = True,
        use_subcarrier_prefilter: bool = False,
    ):
        self.window_size = int(window_size)
        self.step_size = int(step_size)
        self.n_fft = int(n_fft)
        self.fs = max(float(sampling_rate), 1.0)
        self.carrier_freq = float(carrier_freq)
        self.wavelength = SPEED_OF_LIGHT / self.carrier_freq
        self.doppler_limit_hz = float(doppler_limit_hz)
        self.single_antenna = single_antenna

        # Bistatic geometry and PCA configuration
        self.bistatic_angle_deg = float(bistatic_angle_deg)
        self.target_heading_deg = float(target_heading_deg)
        self.bistatic_angle_rad = math.radians(self.bistatic_angle_deg)
        self.target_heading_rad = math.radians(self.target_heading_deg)
        self.use_pca = use_pca
        self.use_subcarrier_prefilter = use_subcarrier_prefilter
        self.last_pca_evr = 0.0

        self.hanning_win = np.hanning(self.window_size)
        self.clutter_filter = StaticClutterFilter(alpha=clutter_alpha)
        self.t0: Optional[float] = None
        self.frame_count: int = 0
        self.frames_since_last_slice: int = self.step_size
        self.slice_counter: int = 0  # Monotonic count of computed slices for UI dirty tracking
        self._lock = threading.RLock()

        # Ring buffer for raw time series: (buffer_len, 64)
        self.time_buffer = deque(maxlen=self.window_size * 2)
        self.sc_buffer = deque(maxlen=self.window_size * 2)
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
        self.bistatic_factor = bistatic_velocity_factor(
            self.wavelength, self.bistatic_angle_rad, self.target_heading_rad
        )
        self.velocity_bins = self.freq_bins * self.bistatic_factor

        # Precompute FFT frequency grid based on current sampling rate
        self.current_fft_freqs = np.fft.fftshift(np.fft.fftfreq(self.n_fft, d=1.0 / self.fs))

        # Last peak metrics
        self.peak_doppler_hz = 0.0
        self.peak_velocity_mps = 0.0

    def update_bistatic_geometry(self, bistatic_angle_deg: float, target_heading_deg: float):
        """Update bistatic angle and target heading angles."""
        with self._lock:
            self.bistatic_angle_deg = float(bistatic_angle_deg)
            self.target_heading_deg = float(target_heading_deg)
            self.bistatic_angle_rad = math.radians(self.bistatic_angle_deg)
            self.target_heading_rad = math.radians(self.target_heading_deg)
            self.bistatic_factor = bistatic_velocity_factor(
                self.wavelength, self.bistatic_angle_rad, self.target_heading_rad
            )
            self.velocity_bins = self.freq_bins * self.bistatic_factor

    def update_band(self, band: int):
        """Update carrier frequency and velocity bins based on wireless band (0=2.4GHz, 1=5GHz)."""
        with self._lock:
            target_freq = CARRIER_FREQ_5G if band == 1 else CARRIER_FREQ_24G
            if abs(target_freq - self.carrier_freq) > 1e6:
                self.carrier_freq = target_freq
                self.wavelength = SPEED_OF_LIGHT / self.carrier_freq
                self.bistatic_factor = bistatic_velocity_factor(
                    self.wavelength, self.bistatic_angle_rad, self.target_heading_rad
                )
                self.velocity_bins = self.freq_bins * self.bistatic_factor

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

            # Optional subcarrier pre-filtering (Hampel/Median) across tones
            if (ENABLE_SUBCARRIER_PREFILTER and self.use_subcarrier_prefilter) and csi_frame is not None:
                csi_frame = prefilter_subcarriers(csi_frame, method=PHASE_FILTER_METHOD)

            # Antenna conjugate cross-correlation or single-antenna fallback
            if (self.single_antenna or ENABLE_SINGLE_ANTENNA) and csi_frame is not None:
                if np.asarray(csi_frame).ndim >= 2 and csi_frame.shape[0] >= 1:
                    csi_ratio = np.asarray(csi_frame[0], dtype=np.complex64)
                else:
                    csi_ratio = np.asarray(csi_frame, dtype=np.complex64)
                if ENABLE_PHASE_FILTERING:
                    csi_ratio = _sanitize_phase_1d(csi_ratio)
            elif csi_frame is not None and np.asarray(csi_frame).ndim >= 2 and csi_frame.shape[0] >= 2:
                h0 = csi_frame[0]
                h1 = csi_frame[1]
                csi_ratio = compute_csi_ratio(h0, h1, method="correlation")
            elif csi_frame is not None and np.asarray(csi_frame).ndim == 1:
                csi_ratio = np.asarray(csi_frame, dtype=np.complex64)
                if ENABLE_PHASE_FILTERING:
                    csi_ratio = _sanitize_phase_1d(csi_ratio)
            else:
                csi_ratio = np.zeros(64, dtype=np.complex64)

            # Apply static clutter filter across time
            dynamic_ratio = self.clutter_filter.filter(csi_ratio)
            self.sc_buffer.append(np.copy(dynamic_ratio))

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
        # DC removal before STFT: Subtract temporal mean per subcarrier over STFT window
        if len(self.sc_buffer) >= self.window_size:
            raw_window = np.array(list(self.sc_buffer)[-self.window_size:])
            if raw_window.ndim == 2 and raw_window.shape[0] == self.window_size:
                if ENABLE_DC_REMOVAL:
                    mean_per_sc = np.mean(raw_window, axis=0, keepdims=True)
                    window_no_dc = raw_window - mean_per_sc
                else:
                    window_no_dc = raw_window

                n_sc = window_no_dc.shape[1]
                if n_sc > 8:
                    sc_start = min(4, n_sc // 8)
                    sc_end = max(sc_start + 1, n_sc - sc_start)
                    active_window = window_no_dc[:, sc_start:sc_end]
                else:
                    active_window = window_no_dc

                # Extract PC1 if PCA enabled, else fall back to subcarrier mean
                if (ENABLE_PCA and self.use_pca) and active_window.shape[1] >= 2:
                    signal_slice, evr = extract_pca_component(active_window)
                    self.last_pca_evr = evr
                elif active_window.shape[1] > 0:
                    signal_slice = np.mean(active_window, axis=1)
                    self.last_pca_evr = 0.0
                else:
                    signal_slice = np.zeros(self.window_size, dtype=np.complex64)
                    self.last_pca_evr = 0.0
            else:
                signal_slice = np.array(list(self.time_buffer)[-self.window_size:])
                if ENABLE_DC_REMOVAL:
                    signal_slice = signal_slice - np.mean(signal_slice)
                self.last_pca_evr = 0.0
        else:
            signal_slice = np.array(list(self.time_buffer)[-self.window_size:])
            if ENABLE_DC_REMOVAL:
                signal_slice = signal_slice - np.mean(signal_slice)
            self.last_pca_evr = 0.0

        if ENABLE_DC_REMOVAL and len(signal_slice) > 0:
            signal_slice = signal_slice - np.mean(signal_slice)

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
        log_spectrum = np.nan_to_num(log_spectrum, nan=-120.0).astype(np.float32)

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
        self.slice_counter += 1

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
        self.src_mac = data["src_mac"]
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

        mac_val = self.src_mac[idx]
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
    """
    High-Performance Matplotlib Real-Time Micro-Doppler GUI Application.
    Engineered for ultra-smooth 30-60 FPS visualization with blitting and instant Windows responsiveness.
    """

    def __init__(
        self,
        source_mode: str = "udp",
        port: int = 5500,
        bind_ip: str = "0.0.0.0",
        file_path: Optional[str] = None,
        speed: float = 1.0,
        single_antenna: bool = False,
        target_fps: int = 30,
        num_doppler_bins: int = 129,
        use_blit: bool = True,
        clim_min: float = -40.0,
        clim_max: float = 20.0,
        bistatic_angle_deg: float = 0.0,
        target_heading_deg: float = 0.0,
        use_pca: bool = True,
    ):
        self.source_mode = source_mode
        self.port = port
        self.bind_ip = bind_ip
        self.file_path = file_path
        self.speed = speed
        self.single_antenna = single_antenna
        self.target_fps = max(10, min(int(target_fps), 60))
        self.use_blit = use_blit
        self.num_doppler_bins = int(num_doppler_bins)
        self.clim_min = float(clim_min)
        self.clim_max = float(clim_max)
        self.bistatic_angle_deg = float(bistatic_angle_deg)
        self.target_heading_deg = float(target_heading_deg)
        self.use_pca = use_pca

        self.running = True
        self.processor = MicroDopplerProcessor(
            single_antenna=self.single_antenna,
            num_doppler_bins=self.num_doppler_bins,
            bistatic_angle_deg=self.bistatic_angle_deg,
            target_heading_deg=self.target_heading_deg,
            use_pca=self.use_pca,
        )

        # Thread-safe buffer for incoming packets
        self.latest_packet: Optional[CSIPacket] = None

        # Statistics
        self.pkt_count = 0
        self.start_time = time.monotonic()
        self.last_rate_time = self.start_time
        self.interval_pkts = 0
        self.current_rate_hz = 0.0

        # UI state tracking & dirty flags
        self._last_rendered_slice_count = -1
        self._current_amp_ylim = 300.0
        self._current_band: Optional[int] = None
        self._last_status_str = ""

        # Preallocated rolling 2D image buffer for spectrogram
        self.spec_buffer_cols = self.processor.spectrogram_history_len
        self.spec_buffer = np.full(
            (len(self.processor.freq_bins), self.spec_buffer_cols),
            self.clim_min,
            dtype=np.float32,
        )

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
            receiver = CSIStreamReceiver(bind_ip=self.bind_ip, port=self.port, timeout=0.1)
        elif self.source_mode == "mock":
            mock_gen = MockCSIGenerator(sample_rate=200.0)
        elif self.source_mode == "file":
            file_reader = CSIPlaybackReader(self.file_path, speed=self.speed)

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
        """Construct multi-panel matplotlib dashboard with blit-ready layout."""
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(12, 7.5), dpi=90)
        self.fig.canvas.manager.set_window_title(
            f"Netgear R6800 MT7615 Micro-Doppler Sensing V2 - [{self.source_mode.upper()}]"
        )

        # 3-row layout: Row 0 (Amplitudes, Phase), Row 1 (Spectrogram), Row 2 (Status banner)
        gs = GridSpec(
            3,
            2,
            figure=self.fig,
            height_ratios=[1.0, 1.35, 0.08],
            hspace=0.42,
            wspace=0.24,
            top=0.94,
            bottom=0.06,
            left=0.08,
            right=0.92,
        )

        # Panel 1: Subcarrier Amplitudes
        self.ax_amp = self.fig.add_subplot(gs[0, 0])
        self.ax_amp.set_title("Subcarrier Amplitudes (Antenna 0 & 1)", fontsize=11, fontweight="bold", pad=8)
        self.ax_amp.set_xlabel("Subcarrier Index (0 - 63)", fontsize=9)
        self.ax_amp.set_ylabel("Amplitude |H|", fontsize=9)
        self.ax_amp.set_xlim(0, 63)
        self.ax_amp.set_ylim(0, self._current_amp_ylim)
        self.ax_amp.grid(True, linestyle="--", alpha=0.3)
        (self.line_amp0,) = self.ax_amp.plot([], [], label="Ant 0 (Rx0)", color="#00ffcc", lw=1.8, animated=self.use_blit)
        (self.line_amp1,) = self.ax_amp.plot([], [], label="Ant 1 (Rx1)", color="#ff007f", lw=1.8, animated=self.use_blit)
        self.ax_amp.legend(loc="upper right", fontsize=8)

        # Panel 2: Phase Sanitization & CSI Ratio
        self.ax_phase = self.fig.add_subplot(gs[0, 1])
        self.ax_phase.set_title("CSI Conjugate Ratio Phase (Sanitized)", fontsize=11, fontweight="bold", pad=8)
        self.ax_phase.set_xlabel("Subcarrier Index (0 - 63)", fontsize=9)
        self.ax_phase.set_ylabel("Phase (radians)", fontsize=9)
        self.ax_phase.set_xlim(0, 63)
        self.ax_phase.set_ylim(-math.pi * 1.5, math.pi * 1.5)
        self.ax_phase.grid(True, linestyle="--", alpha=0.3)
        (self.line_raw_phase,) = self.ax_phase.plot([], [], label="Raw Ratio Phase", color="#888888", lw=1.2, ls=":", animated=self.use_blit)
        (self.line_clean_phase,) = self.ax_phase.plot([], [], label="Sanitized Ratio Phase", color="#ffff00", lw=2.0, animated=self.use_blit)
        self.ax_phase.legend(loc="upper right", fontsize=8)

        # Panel 3: Micro-Doppler Spectrogram (spans full width bottom)
        self.ax_spec = self.fig.add_subplot(gs[1, :])
        self.ax_spec.set_title(
            "Live Micro-Doppler Spectrogram: Time History vs Doppler Shift / Target Velocity",
            fontsize=12,
            fontweight="bold",
            pad=8,
        )
        self.ax_spec.set_xlabel("Time History (seconds, 0 = Now)", fontsize=10)
        self.ax_spec.set_ylabel("Doppler Shift (Hz)", fontsize=10)

        d_lim = self.processor.doppler_limit_hz
        v_max = d_lim * (self.processor.wavelength / 2.0)
        self.time_window_sec = 10.0
        self.ax_spec.set_xlim(-self.time_window_sec, 0.0)
        self.ax_spec.set_ylim(-d_lim, d_lim)

        # Secondary Y axis for Target Velocity (m/s)
        self.ax_vel = self.ax_spec.twinx()
        self.ax_vel.set_ylabel("Target Velocity (m/s)", fontsize=10, color="#ff9900")
        self.ax_vel.tick_params(axis="y", labelcolor="#ff9900")
        self.ax_vel.set_ylim(-v_max, v_max)

        # High-performance image with 'nearest' interpolation (cuts CPU rasterization time by >50%)
        self.im_spec = self.ax_spec.imshow(
            self.spec_buffer,
            aspect="auto",
            origin="lower",
            cmap="inferno",
            extent=[-self.time_window_sec, 0.0, -d_lim, d_lim],
            vmin=self.clim_min,
            vmax=self.clim_max,
            interpolation="nearest",
            animated=self.use_blit,
        )
        self.cbar = self.fig.colorbar(self.im_spec, ax=[self.ax_spec, self.ax_vel], pad=0.08, fraction=0.03)
        self.cbar.set_label("Spectral Power (dB)", fontsize=9)

        # Panel 4: Dedicated status axis (enables blitting without NoneType axes crash)
        self.ax_status = self.fig.add_subplot(gs[2, :])
        self.ax_status.axis("off")
        self.status_text = self.ax_status.text(
            0.5,
            0.5,
            "Rate: 0.0 Hz | Total: 0 pkts | Velocity: 0.00 m/s | Doppler: 0.0 Hz",
            ha="center",
            va="center",
            fontsize=10,
            color="#33ff33",
            fontfamily="monospace",
            bbox=dict(boxstyle="square,pad=0.3", fc="#111111", ec="#33ff33", lw=1.0),
            animated=self.use_blit,
        )

    def _update_plot(self, frame_idx):
        """High-Performance animation update step."""
        needs_redraw = False

        # 1. Update Panel 1 & 2 from the latest incoming CSI packet
        if self.latest_packet is not None:
            pkt = self.latest_packet

            # Dynamic band scaling check (2.4 GHz vs 5 GHz)
            if hasattr(pkt, "band") and pkt.band is not None and pkt.band != self._current_band:
                self._current_band = pkt.band
                self.processor.update_band(pkt.band)
                d_lim = self.processor.doppler_limit_hz
                v_max = d_lim * (self.processor.wavelength / 2.0)
                self.ax_vel.set_ylim(-v_max, v_max)
                needs_redraw = True

            if (self.single_antenna or ENABLE_SINGLE_ANTENNA) and pkt.csi_complex is not None:
                # Single-antenna fallback visualization
                h0 = pkt.csi_complex[0] if np.asarray(pkt.csi_complex).ndim >= 2 else pkt.csi_complex
                amp0 = np.abs(h0)
                x_sc = np.arange(len(amp0))

                self.line_amp0.set_data(x_sc, amp0)
                self.line_amp1.set_data([], [])

                if len(amp0) > 0:
                    max_amp = max(float(np.max(amp0)), 10.0)
                    if max_amp > self._current_amp_ylim * 0.95 or max_amp < self._current_amp_ylim * 0.25:
                        self._current_amp_ylim = max(float(max_amp * 1.5), 50.0)
                        self.ax_amp.set_ylim(0, self._current_amp_ylim)
                        needs_redraw = True

                raw_phase = np.angle(h0)
                sanitized = _sanitize_phase_1d(h0)
                clean_phase = np.angle(sanitized)

                self.line_raw_phase.set_data(x_sc, raw_phase)
                self.line_clean_phase.set_data(x_sc, clean_phase)
            elif pkt.csi_complex is not None and np.asarray(pkt.csi_complex).ndim >= 2 and pkt.csi_complex.shape[0] >= 2:
                h0 = pkt.csi_complex[0]
                h1 = pkt.csi_complex[1]

                # Subcarrier amplitudes
                amp0 = np.abs(h0)
                amp1 = np.abs(h1)
                x_sc = np.arange(len(amp0))

                self.line_amp0.set_data(x_sc, amp0)
                self.line_amp1.set_data(x_sc, amp1)

                # Amplitude headroom with hysteresis (avoids dirtying axis limits on typical frames)
                if len(amp0) > 0 and len(amp1) > 0:
                    max_amp = max(float(np.max(amp0)), float(np.max(amp1)), 10.0)
                    if max_amp > self._current_amp_ylim * 0.95 or max_amp < self._current_amp_ylim * 0.25:
                        self._current_amp_ylim = max(float(max_amp * 1.5), 50.0)
                        self.ax_amp.set_ylim(0, self._current_amp_ylim)
                        needs_redraw = True

                # Ratio & Sanitized Phase
                csi_ratio = compute_csi_ratio(h0, h1)
                raw_ratio_phase = np.angle(csi_ratio)
                sanitized_ratio = _sanitize_phase_1d(csi_ratio)
                clean_phase = np.angle(sanitized_ratio)

                self.line_raw_phase.set_data(x_sc, raw_ratio_phase)
                self.line_clean_phase.set_data(x_sc, clean_phase)

        # 2. Update Panel 3: Spectrogram (fast rolling buffer update only when new STFT slice has arrived)
        curr_slice_count = getattr(self.processor, "slice_counter", len(self.processor.spec_history))
        if curr_slice_count > self._last_rendered_slice_count and len(self.processor.spec_history) > 0:
            num_new = curr_slice_count - self._last_rendered_slice_count
            if self._last_rendered_slice_count == -1:
                num_new = min(len(self.processor.spec_history), self.spec_buffer_cols)
            k = min(num_new, self.spec_buffer_cols)

            # Roll buffer to left and insert new slices at right (0.01 ms operation)
            self.spec_buffer[:, :-k] = self.spec_buffer[:, k:]
            latest_slices = list(self.processor.spec_history)[-k:]
            target_rows = self.spec_buffer.shape[0]

            for idx, s in enumerate(latest_slices):
                s_arr = np.asarray(s).ravel()
                if len(s_arr) == target_rows:
                    self.spec_buffer[:, -k + idx] = s_arr
                elif len(s_arr) > 0:
                    x_old = np.linspace(-self.processor.doppler_limit_hz, self.processor.doppler_limit_hz, len(s_arr))
                    self.spec_buffer[:, -k + idx] = np.interp(self.processor.freq_bins, x_old, s_arr)

            self.im_spec.set_data(self.spec_buffer)
            self._last_rendered_slice_count = curr_slice_count

            if ENABLE_PERCENTILE_SCALING:
                finite_vals = self.spec_buffer[np.isfinite(self.spec_buffer)]
                if len(finite_vals) > 0:
                    non_dc_mask = np.abs(self.processor.freq_bins) > 1.0
                    if np.any(non_dc_mask) and self.spec_buffer.shape[0] == len(self.processor.freq_bins):
                        scale_vals = self.spec_buffer[non_dc_mask, :]
                        scale_finite = scale_vals[np.isfinite(scale_vals)]
                        if len(scale_finite) > 0:
                            finite_vals = scale_finite
                    p10 = float(np.percentile(finite_vals, 10))
                    p99 = float(np.percentile(finite_vals, 99))
                    self.im_spec.set_clim(vmin=p10, vmax=max(p99, p10 + 15.0))

        # 3. Status text update (only redraws text when content actually changes)
        vel = self.processor.peak_velocity_mps
        dop = self.processor.peak_doppler_hz
        rssi_str = ""
        if self.latest_packet:
            rssi_str = f" | RSSI: {self.latest_packet.rssi[:2]}"

        status = (
            f"Rate: {self.current_rate_hz:5.1f} Hz | Total: {self.pkt_count:6d} pkts"
            f"{rssi_str} | Peak Doppler: {dop:+5.1f} Hz | Est Velocity: {vel:+5.2f} m/s"
        )
        if status != self._last_status_str:
            self.status_text.set_text(status)
            self._last_status_str = status

        if needs_redraw and self.fig.canvas:
            self.fig.canvas.draw()

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
        interval_ms = max(int(1000.0 / self.target_fps), 10)
        anim = animation.FuncAnimation(
            self.fig,
            self._update_plot,
            interval=interval_ms,
            blit=self.use_blit,
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
        description="Netgear R6800 MT7615 Micro-Doppler Sensing Visualizer V2"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--udp", action="store_true", help="Stream live from UDP (default)")
    group.add_argument("--file", "-f", help="Replay recorded CSI dataset (.npz file)")
    group.add_argument("--mock", action="store_true", help="Run with simulated walking motion (no router needed)")

    parser.add_argument("--port", "-p", type=int, default=5500, help="UDP port (default: 5500)")
    parser.add_argument("--bind", default="0.0.0.0", help="UDP bind IP (default: 0.0.0.0)")
    parser.add_argument("--speed", type=float, default=1.0, help="File playback speed multiplier (default: 1.0)")
    parser.add_argument("--fps", type=int, default=30, help="Target UI refresh rate in FPS (default: 30)")
    parser.add_argument("--bins", type=int, default=129, help="Number of Doppler frequency bins (default: 129)")
    parser.add_argument("--no-blit", action="store_true", help="Disable blitting (fallback to full redraws)")
    parser.add_argument("--clim-min", type=float, default=-40.0, help="Spectrogram colormap min dB (default: -40.0)")
    parser.add_argument("--clim-max", type=float, default=20.0, help="Spectrogram colormap max dB (default: 20.0)")
    parser.add_argument("--single-antenna", action="store_true", help="Fallback to single-antenna processing (Rx0)")
    parser.add_argument("--bistatic-angle", type=float, default=0.0, help="Bistatic angle beta in degrees (default: 0.0)")
    parser.add_argument("--target-heading", type=float, default=0.0, help="Target heading angle theta in degrees (default: 0.0)")
    parser.add_argument("--no-pca", action="store_true", help="Disable PCA subcarrier aggregation and use scalar mean")

    args = parser.parse_args()

    if args.single_antenna:
        global ENABLE_SINGLE_ANTENNA
        ENABLE_SINGLE_ANTENNA = True

    mode = "mock" if args.mock else ("file" if args.file else "udp")

    print("=" * 68)
    print("  Netgear R6800 MT7615 Micro-Doppler Visualizer V2 Active")
    print("=" * 68)
    print(f"  Mode:           {mode.upper()}")
    if mode == "udp":
        print(f"  UDP Source:     {args.bind}:{args.port}")
    elif mode == "file":
        print(f"  Playback File:  {args.file} (speed: {args.speed}x)")
    elif mode == "mock":
        print("  Simulation:     Human walking & arm swing (+/- 1.4 m/s Doppler)")
    if args.single_antenna:
        print("  Antenna Mode:   Single-antenna fallback (Rx0)")
    print(f"  Target FPS:     {args.fps} FPS")
    print(f"  Doppler Bins:   {args.bins}")
    print(f"  Color Range:    [{args.clim_min:.1f}, {args.clim_max:.1f}] dB")
    print(f"  Bistatic Angle: {args.bistatic_angle:.1f} deg (Heading: {args.target_heading:.1f} deg)")
    print(f"  PCA Subcarrier: {'Disabled (Mean)' if args.no_pca else 'Enabled (PC1 Extraction)'}")
    print(f"  Blitting:       {'Disabled' if args.no_blit else 'Enabled (Hardware Fast)'}")
    print("  Close plot window to exit.")
    print("=" * 68 + "\n")

    app = MicroDopplerApp(
        source_mode=mode,
        port=args.port,
        bind_ip=args.bind,
        file_path=args.file,
        speed=args.speed,
        single_antenna=args.single_antenna,
        target_fps=args.fps,
        num_doppler_bins=args.bins,
        use_blit=(not args.no_blit),
        clim_min=args.clim_min,
        clim_max=args.clim_max,
        bistatic_angle_deg=args.bistatic_angle,
        target_heading_deg=args.target_heading,
        use_pca=(not args.no_pca),
    )
    app.run()


if __name__ == "__main__":
    main()
