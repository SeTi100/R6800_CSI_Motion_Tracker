#!/usr/bin/env python3
"""
IEEE 802.11ac VHT Compressed Beamforming Report (BFR/CFR) Decompressor

Provides mathematical decoding and matrix reconstruction of raw physical Channel State
Information (CSI) transported via 802.11ac Action frames (Category 127: VHT Action,
Action 0: VHT Compressed Beamforming).

Standards Reference:
  - IEEE Std 802.11ac-2013:
      Section 8.4.1.48: VHT Compressed Beamforming
      Section 9.3.3.13: Compressed Beamforming Report
      Section 19.3.12.3: Beamforming feedback & Givens rotation decomposition
"""

import math
import struct
from typing import Tuple, List, Dict, Optional, Union
import numpy as np


class BitReader:
    """Helper to extract arbitrary-width bitfields from a packed byte buffer (LSB-first)."""

    def __init__(self, data: bytes, bit_offset: int = 0):
        self.data = data
        self.bit_offset = bit_offset
        self.total_bits = len(data) * 8

    def read_bits(self, n_bits: int) -> int:
        if self.bit_offset + n_bits > self.total_bits:
            raise ValueError(
                f"BitReader EOF: requested {n_bits} bits at offset {self.bit_offset}, "
                f"total available is {self.total_bits}"
            )
        val = 0
        for i in range(n_bits):
            byte_idx = (self.bit_offset + i) // 8
            bit_idx = (self.bit_offset + i) % 8
            bit = (self.data[byte_idx] >> bit_idx) & 1
            val |= (bit << i)
        self.bit_offset += n_bits
        return val


class BitWriter:
    """Helper to pack arbitrary-width bitfields into a bytearray (LSB-first)."""

    def __init__(self):
        self.data = bytearray()
        self.bit_offset = 0

    def write_bits(self, val: int, n_bits: int):
        for i in range(n_bits):
            bit = (val >> i) & 1
            byte_idx = (self.bit_offset + i) // 8
            bit_idx = (self.bit_offset + i) % 8
            if byte_idx >= len(self.data):
                self.data.append(0)
            self.data[byte_idx] |= (bit << bit_idx)
        self.bit_offset += n_bits

    def get_bytes(self) -> bytes:
        return bytes(self.data)


def get_vht_subcarriers(channel_width: int, grouping: int = 0) -> np.ndarray:
    """
    Returns array of subcarrier indices for the given VHT bandwidth and grouping.
    channel_width: 0=20MHz, 1=40MHz, 2=80MHz, 3=160MHz.
    grouping: 0=Ng=1 (no grouping), 1=Ng=2, 2=Ng=4.
    """
    if channel_width == 0:  # 20 MHz: 52 tones (HT/VHT standard subcarriers)
        tones = np.concatenate([np.arange(-28, -1), np.arange(1, 29)])
        # Valid non-zero data/pilot subcarriers for 20MHz
        tones = np.array([k for k in tones if k not in (-21, -7, 7, 21)])
    elif channel_width == 1:  # 40 MHz: 108 tones
        tones = np.concatenate([np.arange(-58, -2), np.arange(2, 59)])
    elif channel_width == 2:  # 80 MHz: 234 tones
        tones = np.concatenate([np.arange(-122, -2), np.arange(2, 123)])
    else:  # 160 MHz
        tones = np.concatenate([np.arange(-250, -2), np.arange(2, 251)])

    if grouping == 1:  # Ng = 2
        tones = tones[::2]
    elif grouping == 2:  # Ng = 4
        tones = tones[::4]

    return tones


def get_codebook_bits(codebook_info: int) -> Tuple[int, int]:
    """
    Returns (b_psi, b_phi) bit quantization for Givens rotation angles.
    codebook_info: 0 for SU (b_psi=5, b_phi=7 or 2/4 depending on standard subtype),
                   1 for MU (b_psi=7, b_phi=9 or 5/7).
    Standard IEEE 802.11ac Table 9-33:
      codebook=0 (SU): b_psi=5, b_phi=7 (or 4/6)
      codebook=1 (MU): b_psi=7, b_phi=9
    """
    if codebook_info == 0:
        return 5, 7
    else:
        return 7, 9


def construct_givens_matrix(
    n_r: int,
    n_c: int,
    phi_angles: List[float],
    psi_angles: List[float]
) -> np.ndarray:
    """
    Reconstructs the N_r x N_c orthonormal beamforming steering matrix V
    from Givens rotation angles phi and psi according to IEEE 802.11ac Section 19.3.12.3.

    V = [prod_{i=1}^{Nc} D_i(phi) prod_{l=i+1}^{Nr} G_{l,i}(psi)] * I_{Nr x Nc}
    """
    V = np.eye(n_r, dtype=np.complex128)

    angle_idx = 0
    for i in range(n_c):
        # Apply Givens rotations G_{l,i}(psi_{l,i}) and phase rotations D_i(phi_{l,i})
        for l in range(i + 1, n_r):
            phi = phi_angles[angle_idx] if angle_idx < len(phi_angles) else 0.0
            psi = psi_angles[angle_idx] if angle_idx < len(psi_angles) else 0.0
            angle_idx += 1

            # Givens rotation in plane (i, l)
            c = math.cos(psi)
            s = math.sin(psi)

            # Rotation matrix G operating on rows i and l
            G = np.eye(n_r, dtype=np.complex128)
            G[i, i] = c
            G[i, l] = s
            G[l, i] = -s
            G[l, l] = c

            # Phase matrix D on column i
            D = np.eye(n_r, dtype=np.complex128)
            D[l, l] = np.exp(1j * phi)

            # Accumulate rotation V = V @ G.T @ D
            V = V @ G.T @ D

    # Extract first N_c columns
    return V[:, :n_c]


class VHTBeamformingReport:
    """Container for parsed 802.11ac VHT Compressed Beamforming Report data."""

    def __init__(
        self,
        n_c: int,
        n_r: int,
        channel_width: int,
        grouping: int,
        codebook_info: int,
        dialog_token: int,
        avg_snr: List[float],
        subcarrier_indices: np.ndarray,
        v_matrices: Dict[int, np.ndarray],
        snr_per_subcarrier: Optional[Dict[int, np.ndarray]] = None,
    ):
        self.n_c = n_c
        self.n_r = n_r
        self.channel_width = channel_width
        self.grouping = grouping
        self.codebook_info = codebook_info
        self.dialog_token = dialog_token
        self.avg_snr = avg_snr
        self.subcarrier_indices = subcarrier_indices
        self.v_matrices = v_matrices  # map: subcarrier_k -> V (N_r x N_c) complex matrix
        self.snr_per_subcarrier = snr_per_subcarrier or {}

    def get_channel_matrix(self, subcarrier_k: int) -> Optional[np.ndarray]:
        """
        Returns estimated Channel Frequency Response (CFR) matrix H(k) = V(k) * diag(sqrt(SNR))
        for the specified subcarrier.
        """
        if subcarrier_k not in self.v_matrices:
            return None
        V = self.v_matrices[subcarrier_k]
        snr_linear = np.array([10.0 ** (s / 10.0) for s in self.avg_snr[:self.n_c]])
        return V * np.sqrt(snr_linear)


def decompress_vht_bfr(payload: bytes) -> VHTBeamformingReport:
    """
    Decompresses an 802.11ac VHT Compressed Beamforming Report action frame payload.
    Payload starts at the VHT MIMO Control Field (or standard Category 127 Action 0 header).
    """
    offset = 0
    # Strip optional 802.11 Action frame header if present (Category 127, Action 0)
    if len(payload) >= 2 and payload[0] == 127 and payload[1] == 0:
        offset = 2

    if len(payload) - offset < 3:
        raise ValueError("Payload too short for VHT MIMO Control Field (min 3 bytes required)")

    # 1. Parse VHT MIMO Control Field (3 bytes / 24 bits)
    ctrl_bytes = payload[offset:offset + 3]
    ctrl_val = struct.unpack("<I", ctrl_bytes + b"\x00")[0] & 0xFFFFFF
    offset += 3

    n_c = (ctrl_val & 0x7) + 1
    n_r = ((ctrl_val >> 3) & 0x7) + 1
    chan_width = (ctrl_val >> 6) & 0x3
    grouping = (ctrl_val >> 8) & 0x1
    codebook_info = (ctrl_val >> 9) & 0x1
    dialog_token = (ctrl_val >> 12) & 0xF

    b_psi, b_phi = get_codebook_bits(codebook_info)

    # Number of Givens angles per subcarrier: (2 * N_r - 1) * N_c - N_c^2 in general
    # For each column i: (N_r - 1 - i) pairs of (phi, psi)
    n_angles = sum(n_r - 1 - i for i in range(n_c))

    # 2. Parse Average SNR per spatial stream (1 byte each)
    if len(payload) - offset < n_c:
        raise ValueError(f"Payload too short for Average SNR fields: need {n_c} bytes")

    avg_snr = []
    for _ in range(n_c):
        raw_snr = payload[offset]
        offset += 1
        # SNR in dB = (raw / 4) - 10 dB
        avg_snr.append((raw_snr / 4.0) - 10.0)

    # 3. Parse Compressed Beamforming Report angles bitstream
    tones = get_vht_subcarriers(chan_width, grouping)
    bitstream_data = payload[offset:]
    reader = BitReader(bitstream_data)

    v_matrices = {}
    psi_scale = (math.pi / 2.0) / (2.0 ** b_psi)
    phi_scale = (2.0 * math.pi) / (2.0 ** b_phi)

    for tone in tones:
        phi_angles = []
        psi_angles = []
        for _ in range(n_angles):
            phi_raw = reader.read_bits(b_phi)
            psi_raw = reader.read_bits(b_psi)
            phi_angles.append((phi_raw + 0.5) * phi_scale)
            psi_angles.append((psi_raw + 0.5) * psi_scale)

        V_k = construct_givens_matrix(n_r, n_c, phi_angles, psi_angles)
        v_matrices[int(tone)] = V_k

    return VHTBeamformingReport(
        n_c=n_c,
        n_r=n_r,
        channel_width=chan_width,
        grouping=grouping,
        codebook_info=codebook_info,
        dialog_token=dialog_token,
        avg_snr=avg_snr,
        subcarrier_indices=tones,
        v_matrices=v_matrices,
    )


def encode_vht_bfr(
    n_c: int,
    n_r: int,
    channel_width: int,
    grouping: int,
    codebook_info: int,
    dialog_token: int,
    avg_snr: List[float],
    angles_per_subcarrier: Dict[int, Tuple[List[float], List[float]]],
    include_action_header: bool = True
) -> bytes:
    """
    Synthesizes a compliant IEEE 802.11ac VHT Compressed Beamforming Report payload.
    Used for simulation, testing, and benchmark verification.
    """
    out = bytearray()
    if include_action_header:
        out.extend(bytes([127, 0]))  # Category: VHT Action, Action: VHT Compressed Beamforming

    # Pack MIMO Control
    ctrl_val = ((n_c - 1) & 0x7) | (((n_r - 1) & 0x7) << 3)
    ctrl_val |= ((channel_width & 0x3) << 6)
    ctrl_val |= ((grouping & 0x1) << 8)
    ctrl_val |= ((codebook_info & 0x1) << 9)
    ctrl_val |= ((dialog_token & 0xF) << 12)

    out.extend(struct.pack("<I", ctrl_val)[:3])

    # Pack Average SNR per stream
    for snr in avg_snr[:n_c]:
        raw_snr = int(np.clip(round((snr + 10.0) * 4.0), 0, 255))
        out.append(raw_snr)

    # Pack angles
    b_psi, b_phi = get_codebook_bits(codebook_info)
    writer = BitWriter()
    tones = get_vht_subcarriers(channel_width, grouping)

    psi_scale = (math.pi / 2.0) / (2.0 ** b_psi)
    phi_scale = (2.0 * math.pi) / (2.0 ** b_phi)

    for tone in tones:
        phis, psis = angles_per_subcarrier.get(int(tone), ([], []))
        for phi, psi in zip(phis, psis):
            phi_raw = int(np.clip(math.floor(phi / phi_scale), 0, (1 << b_phi) - 1))
            psi_raw = int(np.clip(math.floor(psi / psi_scale), 0, (1 << b_psi) - 1))
            writer.write_bits(phi_raw, b_phi)
            writer.write_bits(psi_raw, b_psi)

    out.extend(writer.get_bytes())
    return bytes(out)
