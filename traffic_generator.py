#!/usr/bin/env python3
"""
Traffic Generator for Netgear R6200 / R6800 Wi-Fi CSI Sensing

Provides continuous packet stream excitation for Wi-Fi Channel State Information
(CSI) capture when the target transmitter (Netgear R6200, Channel 36, 5180 MHz)
does not possess an internal traffic generator.

Supported Modes:
  1. 'probe': Elicits 802.11 Probe Responses from R6200 (BSSID 44:a5:6e:70:e5:8b)
     via serialized iw probe triggers or monitor injection.
  2. 'udp': Streams high-rate UDP datagrams to target IP at a specified packet rate.
  3. 'ping': Pings the target router interface at specified frequency.
  4. 'stats': Continuously monitors /sys/kernel/debug/ieee80211/phy3/mt76/csi_stats
     and computes live packet reception throughput (packets/sec).
"""

import argparse
import socket
import time
import sys
from typing import Optional, Callable

DEFAULT_R6800_IP = "192.168.10.1"
DEFAULT_TARGET_MAC = "44:a5:6e:70:e5:8b"
DEFAULT_SERIAL_PORT = "COM11"
DEFAULT_SERIAL_BAUD = 57600


class TrafficGenerator:
    """Manages excitation traffic generation for CSI capture."""

    def __init__(
        self,
        target_ip: str = DEFAULT_R6800_IP,
        target_port: int = 5500,
        rate_hz: float = 100.0,
        target_mac: str = DEFAULT_TARGET_MAC,
    ):
        self.target_ip = target_ip
        self.target_port = target_port
        self.rate_hz = max(1.0, float(rate_hz))
        self.interval = 1.0 / self.rate_hz
        self.target_mac = target_mac.lower()
        self.running = False
        self.packets_sent = 0

    def generate_udp_stream(self, duration_s: Optional[float] = None, payload_size: int = 64) -> int:
        """
        Transmits a high-rate UDP packet stream to target IP:port.
        Returns total packets transmitted.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = b"X" * payload_size
        self.running = True
        self.packets_sent = 0
        start_time = time.time()

        try:
            while self.running:
                sock.sendto(payload, (self.target_ip, self.target_port))
                self.packets_sent += 1

                if duration_s is not None and (time.time() - start_time) >= duration_s:
                    break

                time.sleep(self.interval)
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
            self.running = False

        return self.packets_sent

    def generate_probe_burst(
        self,
        serial_runner: Optional[Callable[[str, float], str]] = None,
        count: int = 5,
        freq_mhz: int = 5180,
    ) -> int:
        """
        Triggers active 802.11 probe scan requests on the specified frequency
        to elicit immediate Probe Responses from the R6200.
        """
        if serial_runner is None:
            from serial_cmd import run_serial
            serial_runner = run_serial

        probes_sent = 0
        for _ in range(count):
            cmd = f"iw dev phy3-ap0 scan freq {freq_mhz} > /dev/null 2>&1"
            serial_runner(cmd, 1.0)
            probes_sent += 1
            time.sleep(0.1)

        return probes_sent

    @staticmethod
    def parse_csi_stats(stats_text: str) -> dict:
        """Parses /sys/kernel/debug/ieee80211/phyX/mt76/csi_stats into a dictionary."""
        stats = {}
        for line in stats_text.splitlines():
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, val = line.split(":", 1)
            key = key.strip()
            val = val.strip()
            if "/" in val:
                val = val.split("/")[0].strip()
            try:
                stats[key] = int(val)
            except ValueError:
                stats[key] = val
        return stats


def main():
    parser = argparse.ArgumentParser(description="Netgear R6200 / R6800 CSI Traffic Generator")
    parser.add_argument("--mode", choices=["udp", "probe", "stats"], default="stats",
                        help="Traffic generation / monitoring mode (default: stats)")
    parser.add_argument("--ip", default=DEFAULT_R6800_IP, help="Target IP address")
    parser.add_argument("--port", type=int, default=5500, help="Target UDP port")
    parser.add_argument("--rate", type=float, default=100.0, help="Packet transmission rate in Hz")
    parser.add_argument("--duration", type=float, default=None, help="Duration in seconds (default: infinite)")
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT, help="Serial COM port")
    parser.add_argument("--baud", type=int, default=DEFAULT_SERIAL_BAUD, help="Serial baud rate")
    args = parser.parse_args()

    gen = TrafficGenerator(target_ip=args.ip, target_port=args.port, rate_hz=args.rate)

    if args.mode == "udp":
        print(f"[*] Starting UDP traffic generation to {args.ip}:{args.port} at {args.rate:.1f} Hz...")
        sent = gen.generate_udp_stream(duration_s=args.duration)
        print(f"[+] Complete. Sent {sent} packets.")

    elif args.mode == "probe":
        print(f"[*] Triggering 802.11 active probe bursts on 5180 MHz...")
        sent = gen.generate_probe_burst(count=10)
        print(f"[+] Complete. Triggered {sent} probe scan bursts.")

    elif args.mode == "stats":
        print("[*] Monitoring router CSI capture rate over serial...")
        from serial_cmd import run_serial
        prev_captured = None
        prev_time = time.time()

        try:
            while True:
                out = run_serial("cat /sys/kernel/debug/ieee80211/phy3/mt76/csi_stats", 1.0)
                stats = gen.parse_csi_stats(out)
                now = time.time()
                dt = now - prev_time

                if "total_captured" in stats:
                    cur_cap = stats["total_captured"]
                    if prev_captured is not None and dt > 0:
                        rate = (cur_cap - prev_captured) / dt
                        print(f"[CSI Stats] Captured: {cur_cap} | Rate: {rate:6.1f} pkt/s | Drop: {stats.get('total_dropped', 0)}")
                    else:
                        print(f"[CSI Stats] Captured: {cur_cap} | Waiting for rate calculation...")
                    prev_captured = cur_cap
                    prev_time = now
                else:
                    print(f"[!] No valid stats found: {out.strip()[:60]}")

                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[*] Stopped.")


if __name__ == "__main__":
    main()
