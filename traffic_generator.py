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
import struct
import numpy as np
from typing import Optional, Callable

DEFAULT_R6800_IP = "192.168.10.1"
DEFAULT_TARGET_MAC = "44:a5:6e:70:e5:8b"
DEFAULT_SERIAL_PORT = "COM11"
DEFAULT_SERIAL_BAUD = 57600
DEFAULT_ROUTER_PASSWORD = "zanystreet862"


def get_router_runner(
    use_serial: bool = False,
    serial_port: str = DEFAULT_SERIAL_PORT,
    baud: int = DEFAULT_SERIAL_BAUD,
    ip: str = DEFAULT_R6800_IP,
    password: str = DEFAULT_ROUTER_PASSWORD,
) -> Callable[[str, float], str]:
    """Returns a command runner via SSH (preferred over Ethernet) or serial fallback."""
    if not use_serial:
        try:
            import paramiko
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(ip, username="root", password=password, timeout=4)

            def run_ssh(cmd: str, wait: float = 0.5) -> str:
                stdin, stdout, stderr = client.exec_command(cmd)
                return stdout.read().decode("utf-8", errors="replace")

            return run_ssh
        except Exception as e:
            print(f"[*] SSH connection to {ip} not available ({e}); attempting serial...", file=sys.stderr)

    from serial_cmd import run_serial
    return run_serial


def start_remote_extractor(
    ip: str = DEFAULT_R6800_IP,
    laptop_ip: str = "192.168.10.102",
    port: int = 5500,
    password: str = DEFAULT_ROUTER_PASSWORD,
) -> bool:
    """Starts /tmp/csi_extractor quietly in the background via SSH without terminal spam."""
    try:
        import paramiko
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(ip, username="root", password=password, timeout=5)
        client.exec_command("killall csi_extractor 2>/dev/null")
        time.sleep(0.3)
        stdin, stdout, stderr = client.exec_command("ls -d /sys/kernel/debug/ieee80211/phy*/mt76/csi_data 2>/dev/null | tail -1")
        path = stdout.read().decode().strip()
        if not path:
            print("[!] Error: No CSI DebugFS nodes found on router.", file=sys.stderr)
            client.close()
            return False
        parts = path.split("/")
        phy = parts[5] if len(parts) > 5 else "phy5"
        cmd = f"/tmp/csi_extractor -i {phy} -d {laptop_ip} -p {port} -e > /tmp/csi_extractor.log 2>&1 &"
        client.exec_command(cmd)
        time.sleep(0.6)
        stdin, stdout, stderr = client.exec_command("ps | grep csi_extractor | grep -v grep")
        ps_out = stdout.read().decode().strip()
        client.close()
        if ps_out:
            print(f"[+] Successfully launched csi_extractor on {phy} -> {laptop_ip}:{port} (quiet background)")
            return True
        else:
            print("[!] Warning: csi_extractor failed to launch. Check /tmp/csi_extractor.log", file=sys.stderr)
            return False
    except Exception as e:
        print(f"[!] SSH error starting extractor: {e}", file=sys.stderr)
        return False


def stop_remote_extractor(
    ip: str = DEFAULT_R6800_IP,
    password: str = DEFAULT_ROUTER_PASSWORD,
) -> bool:
    """Stops all running csi_extractor and probe loop processes on the router."""
    try:
        import paramiko
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(ip, username="root", password=password, timeout=5)
        client.exec_command("killall csi_extractor 2>/dev/null; killall iw 2>/dev/null")
        client.close()
        print("[+] Stopped csi_extractor and probe loops on router.")
        return True
    except Exception as e:
        print(f"[!] Error stopping extractor: {e}", file=sys.stderr)
        return False


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

    def generate_csi_test_stream(self, duration_s: Optional[float] = None) -> int:
        """
        Transmits valid 1058-byte struct mt76_csi_data UDP packets to verify
        that csi_doppler_V2.py receives and processes packets.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.packets_sent = 0
        start_time = time.time()
        hdr_format = "<QIH6B4b2B6s2B"

        try:
            while self.running:
                now_us = int(time.time() * 1e6)
                seq = self.packets_sent
                # Construct 34-byte header
                hdr = struct.pack(
                    hdr_format,
                    now_us, seq, seq % 4096,
                    1, 0, 36, 4, 1, 64,
                    -45, -50, -55, -60,
                    80, 0, b"\x44\xa5\x6e\x70\xe5\x8b", 0, 0
                )
                # 512 bytes I + 512 bytes Q
                t = seq * self.interval
                sc_amp = 100
                i_arr = np.zeros((4, 64), dtype="<i2")
                q_arr = np.zeros((4, 64), dtype="<i2")
                for a in range(4):
                    phase = 2 * np.pi * 3.0 * t + (a * np.pi / 2)
                    i_arr[a, :] = int(sc_amp * np.cos(phase))
                    q_arr[a, :] = int(sc_amp * np.sin(phase))

                payload = hdr + i_arr.tobytes() + q_arr.tobytes()
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
        continuous: bool = False,
    ) -> int:
        """
        Triggers active 802.11 probe scan requests on the specified frequency
        to elicit immediate Probe Responses from the R6200.
        """
        if serial_runner is None:
            serial_runner = get_router_runner()

        probes_sent = 0
        cmd = f"IFACE=$(iw dev | grep Interface | awk '{{print $2}}' | tail -1); [ -n \"$IFACE\" ] && iw dev $IFACE scan freq {freq_mhz} > /dev/null 2>&1"
        try:
            while True:
                try:
                    serial_runner(cmd, 0.5)
                except Exception as e:
                    print(f"[!] Router command error: {e}", file=sys.stderr)
                    print("[*] Tip: If using PuTTY on COM11, run this loop directly in PuTTY:", file=sys.stderr)
                    print(f"    while true; do {cmd}; sleep 0.05; done &", file=sys.stderr)
                    break
                probes_sent += 1
                if not continuous and probes_sent >= count:
                    break
                time.sleep(self.interval)
        except KeyboardInterrupt:
            pass

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
    parser.add_argument("--mode", choices=["udp", "probe", "stats", "test-udp"], default=None,
                        help="Traffic generation / monitoring mode (default: stats)")
    parser.add_argument("--ip", default=DEFAULT_R6800_IP, help="Target IP address")
    parser.add_argument("--port", type=int, default=5500, help="Target UDP port")
    parser.add_argument("--rate", type=float, default=100.0, help="Packet transmission rate in Hz")
    parser.add_argument("--duration", type=float, default=None, help="Duration in seconds (default: infinite)")
    parser.add_argument("--continuous", action="store_true", help="Keep running continuously until Ctrl+C")
    parser.add_argument("--count", type=int, default=10, help="Number of bursts for probe mode (default: 10)")
    parser.add_argument("--start-extractor", action="store_true", help="Launch /tmp/csi_extractor quietly in background on router")
    parser.add_argument("--stop-extractor", action="store_true", help="Stop all csi_extractor and probe processes on router")
    parser.add_argument("--laptop-ip", default="192.168.10.102", help="Laptop IP for extractor streaming (default: 192.168.10.102)")
    parser.add_argument("--use-serial", action="store_true", help="Force serial COM port instead of SSH")
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT, help="Serial COM port")
    parser.add_argument("--baud", type=int, default=DEFAULT_SERIAL_BAUD, help="Serial baud rate")
    args = parser.parse_args()

    if args.stop_extractor:
        stop_remote_extractor(ip=args.ip)
        return

    if args.start_extractor:
        start_remote_extractor(ip=args.ip, laptop_ip=args.laptop_ip, port=args.port)
        if args.mode is None:
            return

    mode = args.mode or "stats"
    gen = TrafficGenerator(target_ip=args.ip, target_port=args.port, rate_hz=args.rate)
    runner = None
    if mode in ("probe", "stats"):
        runner = get_router_runner(use_serial=args.use_serial, serial_port=args.serial_port, baud=args.baud, ip=args.ip)

    if mode == "test-udp":
        target = args.ip if args.ip != DEFAULT_R6800_IP else "127.0.0.1"
        print(f"[*] Sending valid 1058-byte CSI packets to {target}:{args.port} at {args.rate:.1f} Hz...")
        gen.target_ip = target
        sent = gen.generate_csi_test_stream(duration_s=args.duration)
        print(f"[+] Complete. Sent {sent} test CSI packets.")

    elif args.mode == "udp":
        print(f"[*] Starting UDP traffic generation to {args.ip}:{args.port} at {args.rate:.1f} Hz...")
        sent = gen.generate_udp_stream(duration_s=args.duration)
        print(f"[+] Complete. Sent {sent} packets.")

    elif args.mode == "probe":
        print(f"[*] Triggering 802.11 active probe bursts on 5180 MHz (continuous={args.continuous})...")
        sent = gen.generate_probe_burst(serial_runner=runner, count=args.count, continuous=args.continuous)
        print(f"[+] Complete. Triggered {sent} probe scan bursts.")

    elif args.mode == "stats":
        print("[*] Monitoring router CSI capture rate over SSH/serial...")
        prev_captured = None
        prev_time = time.time()
        stats_cmd = "cat $(ls -d /sys/kernel/debug/ieee80211/phy*/mt76/csi_stats 2>/dev/null | tail -1)"

        try:
            while True:
                out = runner(stats_cmd, 1.0)
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
