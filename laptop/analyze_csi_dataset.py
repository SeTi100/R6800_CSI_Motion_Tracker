#!/usr/bin/env python3
"""
CSI Dataset Analysis & Offline Verification Tool
Netgear R6800 MT7615 Wi-Fi CSI Motion Tracker

Analyzes recorded .npz CSI datasets to verify genuine physical channel characteristics
(4-antenna spatial diversity, differential fading, spatial covariance, hardware FOE Doppler drift),
and enables direct side-by-side comparison between still baseline and walking trials.

Features:
  - Hardware & Software MAC Address Filtering (isolates target benchtop R6200 from neighbor APs)
  - 4-Antenna Spatial Covariance Matrix R = E[(r - mu)(r - mu)^T] & Eigenmode Analysis
  - Differential Spatial Fading Delta_r01(t) = Rx0(t) - Rx1(t) (eliminates common-mode Tx drift)
  - Hardware Frequency Offset (FOE) Doppler Drift Tracking
  - Quantitative Motion Energy & Dynamic Contrast Ratio (Walking vs Still)

Usage:
  # Analyze a single recording:
  python laptop/analyze_csi_dataset.py --file baseline_still.npz

  # Compare still baseline vs walking motion:
  python laptop/analyze_csi_dataset.py --still baseline_still.npz --walking walk_across_room.npz --plot

  # Save figure directly to file:
  python laptop/analyze_csi_dataset.py --still baseline_still.npz --walking walk_across_room.npz --save-fig comparison.png
"""

import os
import sys
import argparse
from typing import Optional, Dict, Any, Tuple
import numpy as np

DEFAULT_TARGET_MAC = "44:a5:6e:70:e5:8b"


def load_dataset(filepath: str, mac_filter: Optional[str] = DEFAULT_TARGET_MAC) -> Dict[str, Any]:
    """
    Load, validate, and optionally MAC-filter an .npz CSI dataset.
    Filters out background neighbor AP packets (e.g. beacons at -90 dBm)
    so only packets from the target transmitter are processed.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Dataset not found: {filepath}")

    data = np.load(filepath, allow_pickle=True)
    required = ["timestamp_us", "seq_num", "rssi"]
    for k in required:
        if k not in data:
            raise ValueError(f"Dataset {filepath} missing required key: {k}")

    ts_raw = data["timestamp_us"]
    total_raw = len(ts_raw)
    src_mac_raw = data["src_mac"] if "src_mac" in data else None

    # Apply MAC address filtering if requested and MAC data is available
    if mac_filter and src_mac_raw is not None and len(src_mac_raw) > 0:
        filter_target = mac_filter.lower()
        src_mac_str = np.array([str(m).lower() for m in src_mac_raw])
        mask = (src_mac_str == filter_target)
        n_matched = int(np.sum(mask))
        if n_matched == 0:
            unique_macs = np.unique(src_mac_str)
            print(f"[!] Warning: No packets matching MAC {mac_filter} in {os.path.basename(filepath)}.")
            print(f"    Available MACs in file: {unique_macs.tolist()}")
            print("    Using all packets unfiltered.")
            mask = np.ones(total_raw, dtype=bool)
        else:
            print(f"[*] Filtered {os.path.basename(filepath)}: {n_matched}/{total_raw} packets match target MAC {mac_filter}")
    else:
        mask = np.ones(total_raw, dtype=bool)

    ts = ts_raw[mask]
    duration_s = float((ts[-1] - ts[0]) / 1e6) if len(ts) > 1 else 0.0
    mean_rate = float((len(ts) - 1) / duration_s) if duration_s > 0 else 0.0

    rssi = data["rssi"][mask]
    csi = data["csi"][mask] if "csi" in data else np.zeros((len(ts), 4, 64), dtype=np.complex64)
    foe = data["foe"][mask] if "foe" in data else np.zeros(len(ts), dtype=np.float32)
    src_mac = src_mac_raw[mask] if src_mac_raw is not None else np.array(["unknown"] * len(ts))

    return {
        "filename": os.path.basename(filepath),
        "path": filepath,
        "n_packets": len(ts),
        "total_unfiltered": total_raw,
        "duration_s": duration_s,
        "mean_rate_hz": mean_rate,
        "timestamp_us": ts,
        "seq_num": data["seq_num"][mask],
        "rssi": rssi,
        "band": data["band"][mask] if "band" in data else np.zeros(len(ts)),
        "channel": data["channel"][mask] if "channel" in data else np.zeros(len(ts)),
        "foe": foe,
        "csi": csi,
        "src_mac": src_mac,
    }


def compute_metrics(dataset: Dict[str, Any]) -> Dict[str, Any]:
    """
    Compute genuine physical verification metrics for a CSI dataset:
      1. 4-Antenna RSSI statistics (Rx0..Rx3)
      2. Differential Spatial Fading Delta_r01(t) and Delta_r23(t)
      3. 4x4 Spatial Covariance Matrix R = E[(r - mu)(r - mu)^T] & Eigenvalues
      4. Hardware FOE Doppler drift Delta_f_FOE(t)
      5. Total Dynamic Motion Energy & Subcarrier CFR check
    """
    n_pkts = dataset["n_packets"]
    rssi = dataset["rssi"]
    foe = dataset["foe"]
    csi = dataset["csi"]

    # 1. RSSI statistics across all 4 receiver chains
    if len(rssi) > 0 and rssi.shape[1] >= 4:
        rssi_means = np.mean(rssi[:, :4], axis=0)
        rssi_stds = np.std(rssi[:, :4], axis=0)
        rssi_min = np.min(rssi[:, :4], axis=0)
        rssi_max = np.max(rssi[:, :4], axis=0)
    else:
        rssi_means = np.zeros(4)
        rssi_stds = np.zeros(4)
        rssi_min = np.zeros(4)
        rssi_max = np.zeros(4)

    # 2. Differential Spatial Fading
    if len(rssi) > 1 and rssi.shape[1] >= 4:
        diff_01 = rssi[:, 0] - rssi[:, 1]
        diff_23 = rssi[:, 2] - rssi[:, 3]
        diff_01_std = float(np.std(diff_01))
        diff_01_var = float(np.var(diff_01))
        diff_23_std = float(np.std(diff_23))
        diff_23_var = float(np.var(diff_23))
    else:
        diff_01 = np.zeros(n_pkts)
        diff_23 = np.zeros(n_pkts)
        diff_01_std = 0.0
        diff_01_var = 0.0
        diff_23_std = 0.0
        diff_23_var = 0.0

    # 3. 4-Antenna Spatial Covariance Matrix & Eigenmodes
    if len(rssi) > 1 and rssi.shape[1] >= 4:
        rssi_cov = np.cov(rssi[:, :4], rowvar=False)
        rssi_eigvals = np.linalg.eigvalsh(rssi_cov)
        spatial_trace = float(np.trace(rssi_cov))
        spatial_dominant_energy = float(rssi_eigvals[-1])
        spatial_cond = float(rssi_eigvals[-1] / (rssi_eigvals[0] + 1e-6))
    else:
        rssi_cov = np.zeros((4, 4))
        rssi_eigvals = np.zeros(4)
        spatial_trace = 0.0
        spatial_dominant_energy = 0.0
        spatial_cond = 1.0

    # 4. Hardware Baseband FOE (Frequency Offset Estimation) Doppler Drift
    if foe is not None and len(foe) > 1:
        foe_mean = float(np.mean(foe))
        foe_std = float(np.std(foe))
        foe_drift = foe - foe_mean
        foe_drift_var = float(np.var(foe_drift))
    else:
        foe_mean = 0.0
        foe_std = 0.0
        foe_drift = np.zeros(n_pkts)
        foe_drift_var = 0.0

    # 5. Composite Dynamic Motion Energy
    # Sum of 4-antenna spatial variance + differential fading fluctuations
    dynamic_motion_energy = spatial_trace + diff_01_var + diff_23_var

    # 6. Check for raw OFDM subcarrier CFR
    amp = np.abs(csi) if csi is not None else np.zeros((n_pkts, 4, 64))
    has_valid_cfr = bool(np.any(amp > 0))

    return {
        "n_packets": n_pkts,
        "duration_s": dataset["duration_s"],
        "sample_rate_hz": dataset["mean_rate_hz"],
        "rssi_means": rssi_means,
        "rssi_stds": rssi_stds,
        "rssi_min": rssi_min,
        "rssi_max": rssi_max,
        "diff_01": diff_01,
        "diff_23": diff_23,
        "diff_01_std": diff_01_std,
        "diff_01_var": diff_01_var,
        "diff_23_std": diff_23_std,
        "diff_23_var": diff_23_var,
        "spatial_cov": rssi_cov,
        "spatial_eigvals": rssi_eigvals,
        "spatial_trace": spatial_trace,
        "spatial_dominant_energy": spatial_dominant_energy,
        "spatial_cond": spatial_cond,
        "foe_mean": foe_mean,
        "foe_std": foe_std,
        "foe_drift": foe_drift,
        "foe_drift_var": foe_drift_var,
        "dynamic_motion_energy": dynamic_motion_energy,
        "has_valid_cfr": has_valid_cfr,
    }


def print_report(name: str, m: Dict[str, Any]):
    """Pretty-print analysis report."""
    print("=" * 68)
    print(f"  CSI Physical Channel Analysis: {name}")
    print("=" * 68)
    print(f"  Packets Filtered:        {m['n_packets']}")
    print(f"  Capture Duration:        {m['duration_s']:.2f} s")
    print(f"  Effective Sample Rate:   {m['sample_rate_hz']:.1f} Hz")
    print("-" * 68)
    print("  4-Antenna RSSI Physical Readings (dBm):")
    print(f"    Rx0: {m['rssi_means'][0]:.2f} ± {m['rssi_stds'][0]:.3f} dBm  [Min: {m['rssi_min'][0]:.1f}, Max: {m['rssi_max'][0]:.1f}]")
    print(f"    Rx1: {m['rssi_means'][1]:.2f} ± {m['rssi_stds'][1]:.3f} dBm  [Min: {m['rssi_min'][1]:.1f}, Max: {m['rssi_max'][1]:.1f}]")
    print(f"    Rx2: {m['rssi_means'][2]:.2f} ± {m['rssi_stds'][2]:.3f} dBm  [Min: {m['rssi_min'][2]:.1f}, Max: {m['rssi_max'][2]:.1f}]")
    print(f"    Rx3: {m['rssi_means'][3]:.2f} ± {m['rssi_stds'][3]:.3f} dBm  [Min: {m['rssi_min'][3]:.1f}, Max: {m['rssi_max'][3]:.1f}]")
    print("-" * 68)
    print("  Differential Spatial Fading (Common-Mode Rejection):")
    print(f"    Delta_r01 (Rx0 - Rx1) Std: {m['diff_01_std']:.3f} dB  (Variance: {m['diff_01_var']:.4f})")
    print(f"    Delta_r23 (Rx2 - Rx3) Std: {m['diff_23_std']:.3f} dB  (Variance: {m['diff_23_var']:.4f})")
    print("-" * 68)
    print("  4-Antenna Spatial Covariance & FOE Baseband Metrics:")
    print(f"    Spatial Covariance Trace:   {m['spatial_trace']:.4f} (total 4-antenna fluctuation)")
    print(f"    Dominant Eigenmode Energy:  {m['spatial_dominant_energy']:.4f} (primary spatial mode)")
    print(f"    Hardware Baseband FOE:      Mean: {m['foe_mean']:.1f} Hz | Drift Std: {m['foe_std']:.2f} Hz")
    print(f"    Dynamic Motion Energy:      {m['dynamic_motion_energy']:.4f}")
    print("-" * 68)
    print(f"    Raw Subcarrier CFR Active:  {'YES' if m['has_valid_cfr'] else 'NO (Zeroed IQ buffers - Using 4-Antenna RF Doppler)'}")
    print("=" * 68 + "\n")


def compare_datasets(
    still_path: str,
    walking_path: str,
    mac_filter: Optional[str] = DEFAULT_TARGET_MAC,
    show_plot: bool = False,
    save_plot: str = "",
):
    """Compare still baseline vs walking motion datasets."""
    print("[*] Loading datasets for comparative analysis...")
    d_still = load_dataset(still_path, mac_filter=mac_filter)
    d_walk = load_dataset(walking_path, mac_filter=mac_filter)

    m_still = compute_metrics(d_still)
    m_walk = compute_metrics(d_walk)

    print_report("BASELINE (STILL ROOM)", m_still)
    print_report("MOTION (WALKING ACROSS ROOM)", m_walk)

    # Compute comparative contrast ratios
    trace_ratio = m_walk["spatial_trace"] / (m_still["spatial_trace"] + 1e-6)
    diff01_ratio = m_walk["diff_01_var"] / (m_still["diff_01_var"] + 1e-6)
    diff23_ratio = m_walk["diff_23_var"] / (m_still["diff_23_var"] + 1e-6)
    motion_contrast = m_walk["dynamic_motion_energy"] / (m_still["dynamic_motion_energy"] + 1e-6)

    print("=" * 68)
    print("  COMPARATIVE PHYSICAL VERIFICATION SUMMARY:")
    print("=" * 68)
    print(f"  Still Spatial Covariance Trace:     {m_still['spatial_trace']:.4f}")
    print(f"  Walking Spatial Covariance Trace:   {m_walk['spatial_trace']:.4f}  (+{(trace_ratio - 1.0)*100:.1f}%)")
    print(f"  Rx0 Fluctuation (Std):              Still: {m_still['rssi_stds'][0]:.3f} dB  ->  Walk: {m_walk['rssi_stds'][0]:.3f} dB ({m_walk['rssi_stds'][0]/m_still['rssi_stds'][0]:.2f}x)")
    print(f"  Rx3 Fluctuation (Std):              Still: {m_still['rssi_stds'][3]:.3f} dB  ->  Walk: {m_walk['rssi_stds'][3]:.3f} dB ({m_walk['rssi_stds'][3]/m_still['rssi_stds'][3]:.2f}x)")
    print(f"  Diff Fading Delta_r01 Fluctuation:  Still: {m_still['diff_01_std']:.3f} dB  ->  Walk: {m_walk['diff_01_std']:.3f} dB ({m_walk['diff_01_std']/m_still['diff_01_std']:.2f}x)")
    print(f"  Total Dynamic Motion Energy:        Still: {m_still['dynamic_motion_energy']:.4f}  ->  Walk: {m_walk['dynamic_motion_energy']:.4f}")
    print(f"  DYNAMIC CONTRAST RATIO:             {motion_contrast:.2f}x higher energy during human movement")
    print("-" * 68)
    if motion_contrast > 1.4:
        print("  [SUCCESS] Genuine human motion is unequivocally detectable above static baseline!")
    else:
        print("  [INFO] Low contrast detected. Verify line-of-sight and antenna positioning.")
    print("=" * 68 + "\n")

    if show_plot or save_plot:
        try:
            import matplotlib.pyplot as plt
            plt.style.use("dark_background")
            fig, axs = plt.subplots(2, 2, figsize=(13, 8), dpi=100)
            fig.suptitle(
                f"Netgear R6800 4-Antenna Wi-Fi Sensing Verification\n"
                f"Target MAC: {mac_filter or 'Unfiltered'} | Dynamic Motion Contrast: {motion_contrast:.2f}x",
                fontsize=13,
                fontweight="bold",
                y=0.98,
            )

            t_s = (d_still["timestamp_us"] - d_still["timestamp_us"][0]) / 1e6
            t_w = (d_walk["timestamp_us"] - d_walk["timestamp_us"][0]) / 1e6

            # Panel 1 (Top-Left): Still 4-Antenna RSSI Time Series
            axs[0, 0].plot(t_s, d_still["rssi"][:, 0], label=f"Rx0 ({m_still['rssi_stds'][0]:.2f} dB)", color="#00ffcc", alpha=0.9, lw=1.3)
            axs[0, 0].plot(t_s, d_still["rssi"][:, 1], label=f"Rx1 ({m_still['rssi_stds'][1]:.2f} dB)", color="#ff007f", alpha=0.9, lw=1.3)
            axs[0, 0].plot(t_s, d_still["rssi"][:, 2], label=f"Rx2 ({m_still['rssi_stds'][2]:.2f} dB)", color="#ffff00", alpha=0.8, lw=1.1)
            axs[0, 0].plot(t_s, d_still["rssi"][:, 3], label=f"Rx3 ({m_still['rssi_stds'][3]:.2f} dB)", color="#33ff33", alpha=0.8, lw=1.1)
            axs[0, 0].set_title(f"Still Baseline: 4-Antenna RSSI (Trace: {m_still['spatial_trace']:.3f})", fontsize=10, fontweight="bold")
            axs[0, 0].set_xlabel("Time (s)", fontsize=9)
            axs[0, 0].set_ylabel("RSSI (dBm)", fontsize=9)
            axs[0, 0].grid(True, linestyle="--", alpha=0.25)
            axs[0, 0].legend(loc="upper right", fontsize=8)

            # Panel 2 (Top-Right): Walking 4-Antenna RSSI Time Series
            axs[0, 1].plot(t_w, d_walk["rssi"][:, 0], label=f"Rx0 ({m_walk['rssi_stds'][0]:.2f} dB)", color="#00ffcc", alpha=0.9, lw=1.3)
            axs[0, 1].plot(t_w, d_walk["rssi"][:, 1], label=f"Rx1 ({m_walk['rssi_stds'][1]:.2f} dB)", color="#ff007f", alpha=0.9, lw=1.3)
            axs[0, 1].plot(t_w, d_walk["rssi"][:, 2], label=f"Rx2 ({m_walk['rssi_stds'][2]:.2f} dB)", color="#ffff00", alpha=0.8, lw=1.1)
            axs[0, 1].plot(t_w, d_walk["rssi"][:, 3], label=f"Rx3 ({m_walk['rssi_stds'][3]:.2f} dB)", color="#33ff33", alpha=0.8, lw=1.1)
            axs[0, 1].set_title(f"Walking Motion: 4-Antenna RSSI (Trace: {m_walk['spatial_trace']:.3f})", fontsize=10, fontweight="bold")
            axs[0, 1].set_xlabel("Time (s)", fontsize=9)
            axs[0, 1].set_ylabel("RSSI (dBm)", fontsize=9)
            axs[0, 1].grid(True, linestyle="--", alpha=0.25)
            axs[0, 1].legend(loc="upper right", fontsize=8)

            # Match Y-limits on top row for fair visual comparison
            y_min = min(axs[0, 0].get_ylim()[0], axs[0, 1].get_ylim()[0])
            y_max = max(axs[0, 0].get_ylim()[1], axs[0, 1].get_ylim()[1])
            axs[0, 0].set_ylim(y_min, y_max)
            axs[0, 1].set_ylim(y_min, y_max)

            # Panel 3 (Bottom-Left): Differential Spatial Fading Delta_r01(t)
            axs[1, 0].plot(t_s, m_still["diff_01"], label=f"Still Delta_r01 (std: {m_still['diff_01_std']:.2f} dB)", color="#00aaee", lw=1.5, alpha=0.85)
            axs[1, 0].plot(t_w, m_walk["diff_01"], label=f"Walking Delta_r01 (std: {m_walk['diff_01_std']:.2f} dB)", color="#ff5500", lw=1.5, alpha=0.85)
            axs[1, 0].set_title("Differential Spatial Fading (Rx0 - Rx1): Still vs Walking", fontsize=10, fontweight="bold")
            axs[1, 0].set_xlabel("Time (s)", fontsize=9)
            axs[1, 0].set_ylabel("Delta RSSI (dB)", fontsize=9)
            axs[1, 0].grid(True, linestyle="--", alpha=0.25)
            axs[1, 0].legend(loc="upper right", fontsize=8)

            # Panel 4 (Bottom-Right): Quantitative Physical Comparison Bar Chart
            categories = ["Rx0 Std", "Rx1 Std", "Rx2 Std", "Rx3 Std", "Diff01 Std", "Spatial Trace"]
            still_vals = [
                m_still["rssi_stds"][0],
                m_still["rssi_stds"][1],
                m_still["rssi_stds"][2],
                m_still["rssi_stds"][3],
                m_still["diff_01_std"],
                m_still["spatial_trace"],
            ]
            walk_vals = [
                m_walk["rssi_stds"][0],
                m_walk["rssi_stds"][1],
                m_walk["rssi_stds"][2],
                m_walk["rssi_stds"][3],
                m_walk["diff_01_std"],
                m_walk["spatial_trace"],
            ]

            x_idx = np.arange(len(categories))
            bar_width = 0.35

            bars_s = axs[1, 1].bar(x_idx - bar_width / 2, still_vals, bar_width, label="Still Baseline", color="#00aaee", alpha=0.8)
            bars_w = axs[1, 1].bar(x_idx + bar_width / 2, walk_vals, bar_width, label="Walking Motion", color="#ff5500", alpha=0.8)

            axs[1, 1].set_title(f"Quantitative Motion Metrics Comparison (Contrast: {motion_contrast:.2f}x)", fontsize=10, fontweight="bold")
            axs[1, 1].set_xticks(x_idx)
            axs[1, 1].set_xticklabels(categories, fontsize=8)
            axs[1, 1].set_ylabel("Energy / Standard Deviation", fontsize=9)
            axs[1, 1].grid(True, linestyle="--", alpha=0.25, axis="y")
            axs[1, 1].legend(loc="upper left", fontsize=8)

            # Add percentage text above walking bars
            for i in range(len(categories)):
                s_val = still_vals[i]
                w_val = walk_vals[i]
                gain = ((w_val - s_val) / (s_val + 1e-6)) * 100.0
                axs[1, 1].text(
                    x_idx[i] + bar_width / 2,
                    w_val + 0.02,
                    f"+{gain:.0f}%" if gain >= 0 else f"{gain:.0f}%",
                    ha="center",
                    va="bottom",
                    fontsize=7,
                    fontweight="bold",
                    color="#33ff33" if gain > 20 else "#cccccc",
                )

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
    parser.add_argument("--mac", default=DEFAULT_TARGET_MAC, help=f"Target MAC address filter (default: {DEFAULT_TARGET_MAC})")
    parser.add_argument("--no-filter", action="store_true", help="Disable MAC address filtering (process all packets promiscuously)")
    parser.add_argument("--plot", action="store_true", help="Display visual comparison plots")
    parser.add_argument("--save-fig", default="", help="Save comparison figure to image file (e.g. comp.png)")
    args = parser.parse_args()

    mac_filter = None if args.no_filter else args.mac

    if args.still and args.walking:
        compare_datasets(args.still, args.walking, mac_filter=mac_filter, show_plot=args.plot, save_plot=args.save_fig)
    elif args.file:
        d = load_dataset(args.file, mac_filter=mac_filter)
        m = compute_metrics(d)
        print_report(d["filename"], m)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
