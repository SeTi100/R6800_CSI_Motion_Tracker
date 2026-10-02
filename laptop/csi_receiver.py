#!/usr/bin/env python3
"""
Netgear R6800 MT7615 CSI UDP Receiver and Logger

Listens for incoming UDP datagrams containing struct mt76_csi_data (1058 bytes)
from the router, parses wireless metadata and complex CSI matrices (4x64 subcarriers),
displays real-time transmission statistics, and logs records to .npz files.

Can be run standalone or imported as a library by csi_doppler.py.
"""

import sys
import time
import socket
import struct
import argparse
import signal
from datetime import datetime
from typing import Optional, Tuple, Dict, Any, List
import numpy as np

# Header format:
#   timestamp_us:  Q (8 bytes, uint64)
#   seq_num:       I (4 bytes, uint32)
#   frame_seq:     H (2 bytes, uint16)
#   band:          B (1 byte, uint8)
#   bw:            B (1 byte, uint8)
#   channel:       B (1 byte, uint8)
#   n_rx:          B (1 byte, uint8)
#   n_tx:          B (1 byte, uint8)
#   n_subcarriers: B (1 byte, uint8)
#   rssi:          4b (4 bytes, 4x int8)
#   noise_floor:   B (1 byte, uint8)
#   _pad0:         B (1 byte, uint8)
#   src_mac:       6s (6 bytes, raw MAC)
#   foe:           h (2 bytes, int16 signed Frequency Offset Estimation)
# Total Header = 34 bytes
CSI_HEADER_FORMAT = "<QIH6B4b2B6sh"
CSI_HEADER_SIZE = 34
CSI_PAYLOAD_SIZE = 1058
CSI_IQ_POINTS = 4 * 64  # 4 antennas x 64 subcarriers = 256 points
CSI_IQ_BYTES = CSI_IQ_POINTS * 2  # 512 bytes each for I and Q

HEADER_STRUCT = struct.Struct(CSI_HEADER_FORMAT)


def format_mac(raw_bytes: bytes) -> str:
    """Format 6 raw bytes into standard colon-separated MAC address."""
    return ":".join(f"{b:02x}" for b in raw_bytes)


class CSIPacket:
    """Represents a single parsed CSI measurement packet."""

    __slots__ = (
        "timestamp_us",
        "seq_num",
        "frame_seq",
        "band",
        "bw",
        "channel",
        "n_rx",
        "n_tx",
        "n_subcarriers",
        "rssi",
        "noise_floor",
        "src_mac",
        "foe",
        "i_data",
        "q_data",
        "csi_complex",
    )

    def __init__(
        self,
        timestamp_us: int,
        seq_num: int,
        frame_seq: int,
        band: int,
        bw: int,
        channel: int,
        n_rx: int,
        n_tx: int,
        n_subcarriers: int,
        rssi: Tuple[int, int, int, int],
        noise_floor: int,
        src_mac: str,
        i_data: np.ndarray,
        q_data: np.ndarray,
        csi_complex: np.ndarray,
        foe: int = 0,
    ):
        self.timestamp_us = timestamp_us
        self.seq_num = seq_num
        self.frame_seq = frame_seq
        self.band = band
        self.bw = bw
        self.channel = channel
        self.n_rx = n_rx
        self.n_tx = n_tx
        self.n_subcarriers = n_subcarriers
        self.rssi = rssi
        self.noise_floor = noise_floor
        self.src_mac = src_mac
        self.foe = foe
        self.i_data = i_data
        self.q_data = q_data
        self.csi_complex = csi_complex

    @classmethod
    def from_bytes(cls, raw: bytes) -> Optional["CSIPacket"]:
        """Parse raw 1058-byte packet into CSIPacket."""
        if len(raw) != CSI_PAYLOAD_SIZE:
            return None

        # Unpack 34-byte header
        header_vals = HEADER_STRUCT.unpack_from(raw, 0)
        (
            ts_us,
            seq_num,
            frame_seq,
            band,
            bw,
            channel,
            n_rx,
            n_tx,
            n_subcarriers,
            r0,
            r1,
            r2,
            r3,
            nf,
            _p0,
            raw_mac,
            foe,
        ) = header_vals

        mac_str = format_mac(raw_mac)
        rssi_tuple = (r0, r1, r2, r3)

        # Parse I and Q matrices (int16 little-endian, 4 antennas x 64 subcarriers)
        i_raw = np.frombuffer(raw, dtype="<i2", count=CSI_IQ_POINTS, offset=CSI_HEADER_SIZE).reshape((4, 64)).copy()
        q_raw = np.frombuffer(raw, dtype="<i2", count=CSI_IQ_POINTS, offset=CSI_HEADER_SIZE + CSI_IQ_BYTES).reshape((4, 64)).copy()

        # Complex frequency response matrix: H = I + j*Q
        csi_complex = i_raw.astype(np.float32) + 1j * q_raw.astype(np.float32)

        return cls(
            timestamp_us=ts_us,
            seq_num=seq_num,
            frame_seq=frame_seq,
            band=band,
            bw=bw,
            channel=channel,
            n_rx=n_rx,
            n_tx=n_tx,
            n_subcarriers=n_subcarriers,
            rssi=rssi_tuple,
            noise_floor=nf,
            src_mac=mac_str,
            i_data=i_raw,
            q_data=q_raw,
            csi_complex=csi_complex,
            foe=foe,
        )


def parse_csi_packet(raw: bytes) -> Optional[CSIPacket]:
    """Helper function to parse a raw CSI packet."""
    return CSIPacket.from_bytes(raw)


class CSIStreamReceiver:
    """Manages UDP socket reception, buffering, and packet loss accounting."""

    def __init__(
        self,
        bind_ip: str = "0.0.0.0",
        port: int = 5500,
        rcvbuf_mb: int = 4,
        timeout: float = 0.5,
        relay_port: Optional[int] = None,
    ):
        self.bind_ip = bind_ip
        self.port = port
        self.rcvbuf_mb = rcvbuf_mb
        self.timeout = timeout
        self.relay_port = relay_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Allow reuse of address
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Set receive buffer
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf_mb * 1024 * 1024)
        except OSError as e:
            print(f"[!] Warning: Unable to set SO_RCVBUF to {rcvbuf_mb}MB: {e}", file=sys.stderr)

        self.sock.settimeout(timeout)
        self.sock.bind((self.bind_ip, self.port))

        # Optional local relay socket
        self.relay_sock = None
        if self.relay_port:
            self.relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Stats
        self.total_received = 0
        self.total_bytes = 0
        self.last_seq: Optional[int] = None
        self.seq_gaps = 0
        self.start_time = time.monotonic()
        self.last_stats_time = self.start_time
        self.interval_pkts = 0
        self.interval_bytes = 0

    def recv_packet(self) -> Optional[CSIPacket]:
        """Receive and parse the next CSI packet. Returns None on socket timeout."""
        try:
            data, _addr = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        except OSError:
            return None

        if len(data) != CSI_PAYLOAD_SIZE:
            return None

        # Relay packet if configured
        if self.relay_sock and self.relay_port:
            try:
                self.relay_sock.sendto(data, ("127.0.0.1", self.relay_port))
            except OSError:
                pass

        pkt = CSIPacket.from_bytes(data)
        if pkt is None:
            return None

        self.total_received += 1
        self.total_bytes += len(data)
        self.interval_pkts += 1
        self.interval_bytes += len(data)

        # Check sequence gaps
        if self.last_seq is not None:
            if pkt.seq_num > self.last_seq:
                if pkt.seq_num > self.last_seq + 1:
                    self.seq_gaps += (pkt.seq_num - (self.last_seq + 1))
                self.last_seq = pkt.seq_num
            elif self.last_seq - pkt.seq_num > 1000:
                # Sequence counter wrap or daemon restart
                self.last_seq = pkt.seq_num
            else:
                # Out-of-order packet arrived; if we previously counted a gap, compensate for it
                if self.seq_gaps > 0:
                    self.seq_gaps -= 1
        else:
            self.last_seq = pkt.seq_num

        return pkt

    def get_interval_stats(self) -> Optional[Dict[str, Any]]:
        """Get throughput stats if 1.0s or more has passed since last check."""
        now = time.monotonic()
        delta = now - self.last_stats_time
        if delta < 1.0:
            return None

        rate_hz = self.interval_pkts / delta
        rate_kb = (self.interval_bytes / 1024.0) / delta
        loss_pct = 0.0
        total_expected = self.total_received + self.seq_gaps
        if total_expected > 0:
            loss_pct = (self.seq_gaps / total_expected) * 100.0

        stats = {
            "rate_hz": rate_hz,
            "rate_kb": rate_kb,
            "total_pkts": self.total_received,
            "seq_gaps": self.seq_gaps,
            "loss_pct": loss_pct,
            "elapsed_s": now - self.start_time,
        }

        self.last_stats_time = now
        self.interval_pkts = 0
        self.interval_bytes = 0
        return stats

    def close(self):
        """Close sockets."""
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.relay_sock:
            try:
                self.relay_sock.close()
            except OSError:
                pass


class CSIDataLogger:
    """Buffers received CSI packets in memory and saves to .npz file."""

    def __init__(self, filename: Optional[str] = None):
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"csi_capture_{timestamp}.npz"
        self.filename = filename

        self.timestamps_us: List[int] = []
        self.seq_nums: List[int] = []
        self.frame_seqs: List[int] = []
        self.band: List[int] = []
        self.bw: List[int] = []
        self.channel: List[int] = []
        self.n_rx: List[int] = []
        self.n_tx: List[int] = []
        self.n_subcarriers: List[int] = []
        self.rssi: List[Tuple[int, int, int, int]] = []
        self.noise_floor: List[int] = []
        self.src_macs: List[str] = []
        self.foe: List[int] = []
        self.i_records: List[np.ndarray] = []
        self.q_records: List[np.ndarray] = []

    def append(self, pkt: CSIPacket):
        """Append a packet to the in-memory buffer."""
        self.timestamps_us.append(pkt.timestamp_us)
        self.seq_nums.append(pkt.seq_num)
        self.frame_seqs.append(pkt.frame_seq)
        self.band.append(pkt.band)
        self.bw.append(pkt.bw)
        self.channel.append(pkt.channel)
        self.n_rx.append(pkt.n_rx)
        self.n_tx.append(pkt.n_tx)
        self.n_subcarriers.append(pkt.n_subcarriers)
        self.rssi.append(pkt.rssi)
        self.noise_floor.append(pkt.noise_floor)
        self.src_macs.append(pkt.src_mac)
        self.foe.append(pkt.foe)
        self.i_records.append(pkt.i_data)
        self.q_records.append(pkt.q_data)

    def count(self) -> int:
        return len(self.timestamps_us)

    def save(self) -> str:
        """Commit records to compressed .npz file."""
        n = len(self.timestamps_us)
        if n == 0:
            print("[i] No packets to save.")
            return self.filename

        print(f"[+] Packing {n} CSI records for export...")
        timestamps = np.array(self.timestamps_us, dtype=np.uint64)
        seq_nums = np.array(self.seq_nums, dtype=np.uint32)
        frame_seqs = np.array(self.frame_seqs, dtype=np.uint16)
        band = np.array(self.band, dtype=np.uint8)
        bw = np.array(self.bw, dtype=np.uint8)
        channel = np.array(self.channel, dtype=np.uint8)
        n_rx = np.array(self.n_rx, dtype=np.uint8)
        n_tx = np.array(self.n_tx, dtype=np.uint8)
        n_subcarriers = np.array(self.n_subcarriers, dtype=np.uint8)
        rssi = np.array(self.rssi, dtype=np.int8)
        noise_floor = np.array(self.noise_floor, dtype=np.uint8)
        src_macs = np.array(self.src_macs)
        foe_array = np.array(self.foe, dtype=np.int16)

        i_array = np.stack(self.i_records, axis=0)  # Shape (N, 4, 64)
        q_array = np.stack(self.q_records, axis=0)  # Shape (N, 4, 64)
        csi_complex = i_array.astype(np.float32) + 1j * q_array.astype(np.float32)

        np.savez_compressed(
            self.filename,
            timestamp_us=timestamps,
            seq_num=seq_nums,
            frame_seq=frame_seqs,
            band=band,
            bw=bw,
            channel=channel,
            n_rx=n_rx,
            n_tx=n_tx,
            n_subcarriers=n_subcarriers,
            rssi=rssi,
            noise_floor=noise_floor,
            src_mac=src_macs,
            foe=foe_array,
            i_data=i_array,
            q_data=q_array,
            csi=csi_complex,
        )
        print(f"[OK] Successfully saved {n} records to {self.filename}")
        return self.filename


def main():
    parser = argparse.ArgumentParser(
        description="Netgear R6800 MT7615 CSI UDP Receiver and Logger"
    )
    parser.add_argument("--bind", default="0.0.0.0", help="IP address to bind UDP socket (default: 0.0.0.0)")
    parser.add_argument("--port", "-p", type=int, default=5500, help="UDP port to listen on (default: 5500)")
    parser.add_argument("--rcvbuf", type=int, default=4, help="Socket receive buffer in MB (default: 4)")
    parser.add_argument("--output", "-o", help="Output .npz file (default: csi_capture_<timestamp>.npz)")
    parser.add_argument("--no-save", action="store_true", help="Do not save packets to disk (monitor only)")
    parser.add_argument("--max-packets", "-n", type=int, default=0, help="Stop after N packets (0 = infinite)")
    parser.add_argument("--relay-port", type=int, help="Locally relay received packets to 127.0.0.1:<PORT>")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print details for every received packet")
    args = parser.parse_args()

    receiver = CSIStreamReceiver(
        bind_ip=args.bind,
        port=args.port,
        rcvbuf_mb=args.rcvbuf,
        relay_port=args.relay_port,
    )

    logger = None if args.no_save else CSIDataLogger(filename=args.output)

    running = True

    def sig_handler(sig, frame):
        nonlocal running
        print("\n[*] Stopping receiver...")
        running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print("=" * 65)
    print("  Netgear R6800 MT7615 CSI Receiver Engine Active")
    print("=" * 65)
    print(f"  Listening on:   {args.bind}:{args.port} (UDP)")
    print(f"  Receive Buffer: {args.rcvbuf} MB")
    print(f"  Output File:    {'Disabled' if args.no_save else logger.filename}")
    if args.relay_port:
        print(f"  Local Relay:    127.0.0.1:{args.relay_port}")
    print("  Press Ctrl+C to terminate and save.")
    print("=" * 65 + "\n")

    try:
        while running:
            pkt = receiver.recv_packet()
            if pkt is not None:
                if logger:
                    logger.append(pkt)

                if args.verbose:
                    print(
                        f"[V] Seq: {pkt.seq_num:6d} | MAC: {pkt.src_mac} | "
                        f"RSSI: [{pkt.rssi[0]:3d}, {pkt.rssi[1]:3d}, {pkt.rssi[2]:3d}, {pkt.rssi[3]:3d}] dBm | "
                        f"BW: {pkt.bw} | Ch: {pkt.channel:3d} | Ant: {pkt.n_rx}Rx"
                    )

                if args.max_packets > 0 and receiver.total_received >= args.max_packets:
                    print(f"[*] Reached target limit of {args.max_packets} packets.")
                    break

            # Print throughput stats
            stats = receiver.get_interval_stats()
            if stats is not None:
                print(
                    f"[RX] Rate: {stats['rate_hz']:6.1f} pkt/s ({stats['rate_kb']:6.1f} KB/s) | "
                    f"Total: {stats['total_pkts']:7d} | Lost: {stats['seq_gaps']:4d} ({stats['loss_pct']:4.1f}%)"
                )
                sys.stdout.flush()

    finally:
        receiver.close()
        if logger and logger.count() > 0:
            logger.save()

    print("\n--- CSI Receiver Finished ---")
    print(f"  Total Packets Received: {receiver.total_received}")
    print(f"  Total Data Received:    {receiver.total_bytes / (1024.0 * 1024.0):.2f} MB")
    print(f"  Detected Sequence Gaps: {receiver.seq_gaps}")


if __name__ == "__main__":
    main()
