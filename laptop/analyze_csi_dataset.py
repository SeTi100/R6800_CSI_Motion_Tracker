#!/usr/bin/env python3
"""
CSI Dataset Analysis & Offline Verification Tool
Netgear R6800 MT7615 Wi-Fi CSI Motion Tracker

Analyzes recorded .npz CSI datasets to verify genuine physical channel characteristics
(multipath frequency-selectivity, static clutter suppression, dynamic Doppler energy),
and enables direct side-by-side comparison between still baseline and walking trials.

Usage:
  # Analyze a single recording:
  python laptop/analyze_csi_dataset.py --file baseline_still.npz

  # Compare still baseline vs walking motion:
  python laptop/analyze_csi_dataset.py --still baseline_still.npz --walking walk_across_room.npz --plot
"""

import os
import sys
import argparse
import numpy as np


def load_dataset(filepath: str) -> dict:
    """Load and validate an .npz CSI dataset."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Dataset not found: {filepath}")

    data = np.load(filepath, allow_pickle=True)
    required = ["timestamp_us", "seq_num", "rssi", "csi"]
    for k in required:
        if k not in data:
            raise ValueError(f"Dataset {filepath} missing required key: {k}")

    ts = data["timestamp_us"]
    csi = data["csi"]
    duration_s = (ts[-1] - ts[0]) / 1e6 if len(ts) > 1 else 0.0
    mean_rate = (len(ts) - 1) / duration_s if duration_s > 0 else 0.0

    return {
        "filename": os.path.basename(filepath),
        "path": filepath,
        "n_packets": len(ts),
        "duration_s": duration_s,
        "mean_rate_hz": mean_rate,
        "timestamp_us": ts,
        "seq_num": data["seq_num"],
        "rssi": data["rssi"],
        "band": data["band"] if "band" in data else np.zeros(len(ts)),
        "channel": data["channel"] if "channel" in data else np.zeros(len(ts)),
        "csi": csi,  # Shape (N, 4, 64) complex
    }


def compute_metrics(dataset: dict) -> dict:
    """Compute physical verification metrics for a CSI dataset."""
    csi = dataset["csi"]  # (N, N_ant, N_subcarriers)
    n_pkts = len(csi)

    # 1. RSSI metrics per antenna
    rssi = dataset["rssi"]
    rssi_means = np.mean(rssi, axis=0) if len(rssi) > 0 else np.zeros(4)
    rssi_stds = np.std(rssi, axis=0) if len(rssi) > 0 else np.zeros(4)

    # 2. Subcarrier Amplitude Frequency Selectivity
    # Physical multipath exhibits irregular tone-to-tone variations (peaks and fading notches).
    amp = np.abs(csi)  # (N, 4, 64)
    # Mean amplitude spectrum across subcarriers for Rx0
    rx0_amp_mean = np.mean(amp[:, 0, :], axis=0) if amp.shape[1] > 0 else np.zeros(64)
    rx0_amp_std_over_tones = np.std(rx0_amp_mean)
    rx0_amp_mean_val = np.mean(rx0_amp_mean)
    # Frequency selectivity ratio (higher = more multipath fading diversity across tones)
    freq_selectivity = (rx0_amp_std_over_tones / (rx0_amp_mean_val + 1e-6)) if rx0_amp_mean_val > 0 else 0.0

    # 3. Dynamic Motion Energy Ratio (Doppler power distribution)
    # Compute antenna conjugate product H0 * conj(H1)
    if amp.shape[1] >= 2:
        cross = csi[:, 0, :] * np.conj(csi[:, 1, :])
    else:
        cross = csi[:, 0, :]

    # Static clutter removal: subtract temporal mean per subcarrier
    cross_detrend = cross - np.mean(cross, axis=0, keepdims=True)

    # Total signal variance (temporal fluctuation energy)
    temporal_variance = np.mean(np.var(np.abs(cross), axis=0))
    dynamic_motion_energy = np.mean(np.var(cross_detrend, axis=0))

    # 4. Zero CFR detection (checks if data is blanked/zeroed)
    is_all_zeros = np.all(amp == 0.0)
    has_valid_cfr = not is_all_zeros and np.any(amp > 0.0)

    return {
        "n_packets": n_pkts,
        "duration_s": dataset["duration_s"],
        "sample_rate_hz": dataset["mean_rate_hz"],
        "rssi_means": rssi_means,
        "rssi_stds": rssi_stds,
        "has_valid_cfr": has_valid_cfr,
        "freq_selectivity": freq_selectivity,
        "temporal_variance": temporal_variance,
        "dynamic_motion_energy": dynamic_motion_energy,
        "rx0_spectrum": rx0_amp_mean,
    }


def print_report(name: str, m: dict):
    """Pretty-print analysis report."""
    print("=" * 65)
    print(f"  CSI Dataset Analysis: {name}")
    print("=" * 65)
    print(f"  Packets Captured:        {m['n_packets']}")
    print(f"  Capture Duration:        {m['duration_s']:.2f} s")
    print(f"  Effective Sample Rate:   {m['sample_rate_hz']:.1f} Hz")
    print(f"  Antenna RSSI (dBm):      Rx0: {m['rssi_means'][0]:.1f} ± {m['rssi_stds'][0]:.2f} dBm")
    if len(m['rssi_means']) > 1:
        print(f"                           Rx1: {m['rssi_means'][1]:.1f} ± {m['rssi_stds'][1]:.2f} dBm")
    print("-" * 65)
    print("  Physical Channel Diagnostics:")
    print(f"  Valid Non-Zero CFR:      {'YES' if m['has_valid_cfr'] else 'NO (Zeroed IQ buffers)'}")
    print(f"  Frequency Selectivity:   {m['freq_selectivity']:.3f} (tone-to-tone multipath variation)")
    print(f"  Temporal Motion Energy:  {m['dynamic_motion_energy']:.3f} (dynamic fluctuation power)")
    print("-" * 65)
    if not m['has_valid_cfr']:
        print("  [!] Note: Subcarrier I/Q values are currently zero in this dataset.")
        print("      To capture complex CFR matrices, activate VHT BFR action frames")
        print("      using 'python laptop/vht_bfr_decompressor.py'.")
    elif m['dynamic_motion_energy'] > 10.0:
        print("  [+] Assessment: High dynamic fluctuation detected (Walking / Locomotion).")
    else:
        print("  [+] Assessment: Low dynamic fluctuation detected (Static Baseline / Still Room).")
    print("=" * 65 + "\n")


def compare_datasets(still_path: str, walking_path: str, show_plot: bool = False, save_plot: str = ""):
    """Compare still baseline vs walking motion datasets."""
    print("[*] Loading datasets for comparative analysis...")
    d_still = load_dataset(still_path)
    d_walk = load_dataset(walking_path)

    m_still = compute_metrics(d_still)
    m_walk = compute_metrics(d_walk)

    print_report("BASELINE (STILL)", m_still)
    print_report("MOTION (WALKING)", m_walk)

    # Compute comparative contrast
    if m_still['dynamic_motion_energy'] > 0:
        contrast_ratio = m_walk['dynamic_motion_energy'] / (m_still['dynamic_motion_energy'] + 1e-6)
    else:
        contrast_ratio = m_walk['dynamic_motion_energy']

    print("=" * 65)
    print("  Comparative Physical Verification Summary:")
    print("=" * 65)
    print(f"  Still Motion Energy:     {m_still['dynamic_motion_energy']:.4f}")
    print(f"  Walking Motion Energy:   {m_walk['dynamic_motion_energy']:.4f}")
    print(f"  Dynamic Contrast Ratio:  {contrast_ratio:.1f}x higher during movement")
    if contrast_ratio > 2.0:
        print("  [SUCCESS] Motion presence is clearly distinguishable above static baseline.")
    else:
        print("  [INFO] Fluctuation contrast is low. Check antenna alignment or client traffic rate.")
    print("=" * 65 + "\n")

    if show_plot or save_plot:
        try:
            import matplotlib.pyplot as plt
            fig, axs = plt.subplots(2, 2, figsize=(12, 7))
            fig.suptitle("CSI Offline Physical Analysis: Still Baseline vs Walking Motion", fontsize=13, fontweight="bold")

            # 1. RSSI Comparison
            t_s = (d_still["timestamp_us"] - d_still["timestamp_us"][0]) / 1e6
            t_w = (d_walk["timestamp_us"] - d_walk["timestamp_us"][0]) / 1e6

            axs[0, 0].plot(t_s, d_still["rssi"][:, 0], label="Still Rx0", color="#00aaee", alpha=0.8)
            axs[0, 0].set_title("Baseline RSSI Stability", fontsize=10, fontweight="bold")
            axs[0, 0].set_xlabel("Time (s)")
            axs[0, 0].set_ylabel("RSSI (dBm)")
            axs[0, 0].grid(True, linestyle="--", alpha=0.3)
            axs[0, 0].legend()

            axs[0, 1].plot(t_w, d_walk["rssi"][:, 0], label="Walking Rx0", color="#ff5500", alpha=0.8)
            axs[0, 1].set_title("Walking RSSI Fluctuation", fontsize=10, fontweight="bold")
            axs[0, 1].set_xlabel("Time (s)")
            axs[0, 1].set_ylabel("RSSI (dBm)")
            axs[0, 1].grid(True, linestyle="--", alpha=0.3)
            axs[0, 1].legend()

            # 2. Subcarrier Amplitude Profiles
            axs[1, 0].plot(m_still["rx0_spectrum"], color="#00aaee", lw=1.8, label="Still Spectrum")
            axs[1, 0].set_title("Still Subcarrier Amplitude Profile", fontsize=10, fontweight="bold")
            axs[1, 0].set_xlabel("Subcarrier Index (0-63)")
            axs[1, 0].set_ylabel("Mean Amplitude")
            axs[1, 0].grid(True, linestyle="--", alpha=0.3)
            axs[1, 0].legend()

            axs[1, 1].plot(m_walk["rx0_spectrum"], color="#ff5500", lw=1.8, label="Walking Spectrum")
            axs[1, 1].set_title("Walking Subcarrier Amplitude Profile", fontsize=10, fontweight="bold")
            axs[1, 1].set_xlabel("Subcarrier Index (0-63)")
            axs[1, 1].set_ylabel("Mean Amplitude")
            axs[1, 1].grid(True, linestyle="--", alpha=0.3)
            axs[1, 1].legend()

            plt.tight_layout()
            if save_plot:
                plt.savefig(save_plot, dpi=150)
                print(f"[OK] Saved comparison plot to: {save_plot}")
            if show_plot:
                plt.show()
        except ImportError:
            print("[!] Matplotlib not installed; skipping plot generation.")


def main():
    parser = argparse.ArgumentParser(description="Netgear R6800 CSI Dataset Analysis & Verification Tool")
    parser.add_argument("--file", "-f", help="Path to single .npz CSI dataset to analyze")
    parser.add_argument("--still", help="Path to baseline still .npz dataset for comparison")
    parser.add_argument("--walking", help="Path to walking motion .npz dataset for comparison")
    parser.add_argument("--plot", action="store_true", help="Display visual comparison plots")
    parser.add_argument("--save-fig", default="", help="Save comparison figure to image file (e.g. comp.png)")
    args = parser.parse_args()

    if args.still and args.walking:
        compare_datasets(args.still, args.walking, show_plot=args.plot, save_plot=args.save_fig)
    elif args.file:
        d = load_dataset(args.file)
        m = compute_metrics(d)
        print_report(d["filename"], m)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
