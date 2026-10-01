# Netgear R6800 (MT7615E) CSI Sensing: System Architecture, Ground Truth & Next Steps

> **Status as of October 2026**  
> **Target:** Netgear R6800 (MediaTek MT7621AT SoC, 2x MT7615E PCIe 4x4:4)  
> **Environment:** OpenWrt 25.12.2 (Linux Kernel 6.12.74, `mipsel_24kc_musl`)  
> **Primary Goal:** Real-Time Micro-Doppler Wi-Fi Human Sensing (Respiration, Motion, Gait)

---

## 1. Executive Summary & Ground Truth: The Fundamental Showstopper

### 1.1 What Is Currently Working (100% Functional)
1. **Network Link & Wi-Fi AP:**
   * 5 GHz Access Point `openwrt_home` is active on Channel 36 (VHT80, 5180 MHz).
   * Subnet is cleanly isolated on `192.168.10.1/24` with DHCP serving `192.168.10.x`.
   * Offline Wi-Fi connection from a Windows laptop without Ethernet is fully verified.
2. **Kernel Infrastructure & Ring Buffer:**
   * Custom patches (`900`–`907`) compile with zero errors and load cleanly into Linux 6.12.74.
   * `907-fix-napi-pagepool-teardown.patch` resolves the NAPI page-pool assertion during module hot-reloading (`rmmod`).
   * DebugFS nodes are active under `/sys/kernel/debug/ieee80211/phy5/mt76/csi_*`.
   * A 256-slot ring buffer (`dev->mt76.csi_buf`) handles multi-threaded capture with spinlocks and a waitqueue.
3. **Userspace Streaming Daemon (`csi_extractor`):**
   * Cross-compiled MIPS32r2 ELF binary runs on the router with near-zero CPU usage (`poll()` event loop).
   * Transmits 1058-byte binary datagrams over UDP to laptop port 5500.
4. **Laptop Pipeline Foundation:**
   * Python receiver and Matplotlib dashboard with 4 panels exist and run without crashes.

---

### 1.2 The Showstopper: Synthetic Pseudo-CSI in Kernel Patch 902

In [`patches/902-csi-mt7615-rx-capture.patch`](file:///C:/Users/NoSet/Coding_Projects/R6800_CSI/patches/902-csi-mt7615-rx-capture.patch), the driver hook inside `mt7615_mac_fill_rx()` currently does the following:

```c
/* CURRENT KERNEL HOOK IMPLEMENTATION (PSEUDO-CSI) */
foe_val = (s16)FIELD_GET(MT_RXV5_FOE, le32_to_cpu(rxd[4]));
if (foe_val & BIT(11))
    foe_val -= 4096;

for (ant = 0; ant < MT76_CSI_MAX_ANTENNAS; ant++) {
    s8 ant_rssi = csi_rec->rssi[ant];
    s32 amp = (ant_rssi > -110 && ant_rssi <= 0) ? (ant_rssi + 110) * 3 : 0;
    s16 ant_phase_offset = ant * 128;

    for (sc = 0; sc < MT76_CSI_MAX_SUBCARRIERS; sc++) {
        s32 sc_factor = 256 + ((sc - 32) * (sc - 32) / 4);
        s32 sc_amp = (amp * sc_factor) >> 8;
        s16 phase = (sc * 32 + ant_phase_offset + (foe_val >> 2)) & 0x1ff;
        /* Sine / Cosine approximation generates i_data and q_data */
        csi_rec->i_data[ant][sc] = (s16)((sc_amp * cos_v) >> 7);
        csi_rec->q_data[ant][sc] = (s16)((sc_amp * sin_v) >> 7);
    }
}
```

#### Why This Fails Physical Sensing:
* **`FOE` (Frequency Offset Estimation)** in RXV5 is a **single scalar value** estimated by hardware from the preamble for the entire packet.
* **`RCPI` (RSSI)** in RXV4 is a **broadband scalar power value** across the entire 20/40/80 MHz channel for each antenna.
* Synthesizing 64 subcarriers from scalar FOE and scalar RSSI produces 64 mathematically coupled, identical sinusoidal waves with artificial offsets.
* **Result:** There is **zero real Channel Frequency Response (CFR)**. The frequency-selective multipath fading (the constructive and destructive interference caused by human bodies reflecting Wi-Fi signals) is completely absent.
* **What the visualizer currently sees:** Not human movement, but thermal clock drift of the crystal oscillator (`FOE`) and bulk RSSI noise.

---

## 2. Hardware Architecture: MT7615 vs. MT7915

Understanding MediaTek hardware generations is essential for any developer working on this project:

```
+-----------------------------------------------------------------------------------------+
| Generation 1: MT7615 (802.11ac Wave 2 / Wi-Fi 5) - Netgear R6800                        |
| - Co-Processors: Andes N9 (Firmware) + ARM Cortex-R4 (MAC/BB offload)                   |
| - DMA Descriptors: Group 3 RX Vectors (RXV1..RXV6)                                      |
| - RXV contains: RCPI0..3, FOE, Noise Floor (NF0), Modulation, Frame Mode, WCID          |
| - Native CFR: Not exposed in default RX descriptors! CFR requires either:              |
|     (a) 802.11ac Explicit Compressed Beamforming Feedback Reports (BFR/CFR)             |
|     (b) Diagnostic / ATE baseband capture via MCU commands / testmode buffers            |
+-----------------------------------------------------------------------------------------+
| Generation 2: MT7915 / MT7916 (802.11ax / Wi-Fi 6)                                      |
| - Introduced native MCU_EXT_CMD_CSI_CTRL (0xc2) firmware streaming                      |
| - Hardware DMA ring directly streams raw complex CFR matrices into host memory          |
+-----------------------------------------------------------------------------------------+
```

### The Two Legitimate Paths to Real CFR on MT7615:

1. **Path A: 802.11ac Explicit Compressed Beamforming Reports (BFR):**
   * The 802.11ac standard specifies Channel State Information feedback via Null Data Packet Announcement (NDPA) and Null Data Packet (NDP) sounding.
   * When an AP or client sends an NDP sounding frame, the receiver's baseband hardware calculates the steering matrix $V$ and SNR per subcarrier and transmits an Action frame (`Category 127: VHT Action`, `Action 0: VHT Compressed Beamforming`).
   * By tapping or injecting NDPA/NDP frames, raw channel matrix measurements can be harvested directly from standard 802.11ac frames without non-standard firmware hacks.

2. **Path B: Andes N9 Firmware Diagnostics / ATE Test Hook:**
   * In the OpenWrt driver source, `mt7615_mac_fill_tm_rx()` handles Testmode / ATE RX.
   * MediaTek internal tools capture baseband samples via calibration registers (e.g. `MT_WF_PHY_RFINTF3` or vendor MCU calibration commands).
   * Reverse-engineering the Andes N9 binary firmware (`mt7615_n9.bin`, 20200814163649) to locate raw CFR dump routines.

---

## 3. Binary Wire Protocol: `struct mt76_csi_data`

The streaming protocol between router and laptop is a fixed 1058-byte binary packet transmitted via UDP.

### Header (34 Bytes, Little-Endian):
```
Offset  Size  Field          Type     Description
---------------------------------------------------------------------------------------------
0x00    8B    timestamp_us   uint64   Monotonic kernel timestamp (ktime_to_us)
0x08    4B    seq_num        uint32   Monotonic packet counter from driver ring buffer
0x0C    2B    frame_seq      uint16   802.11 Sequence Number (IEEE80211_SEQ_TO_SN)
0x0E    1B    band           uint8    0 = 2.4 GHz, 1 = 5 GHz
0x0F    1B    bw             uint8    0 = 20 MHz, 1 = 40 MHz, 2 = 80 MHz (MT_RXV1_FRAME_MODE)
0x10    1B    channel        uint8    Primary channel frequency (e.g. 36 / 5180 MHz)
0x11    1B    n_rx           uint8    Number of active RX chains (1..4)
0x12    1B    n_tx           uint8    Number of spatial streams (1..4)
0x13    1B    n_subcarriers  uint8    Subcarrier count (currently 64)
0x14    4B    rssi[4]        int8[4]  Physical per-antenna RSSI in dBm (-110..0 dBm)
0x18    1B    noise_floor    uint8    Noise floor in dBm (MT_RXV6_NF0)
0x19    1B    _pad0          uint8    Structure alignment padding
0x1A    6B    src_mac[6]     uint8[6] Transmitter MAC address
0x20    2B    _pad1[2]       uint8[2] Structure alignment padding
```

### Payload (1024 Bytes):
```
Offset  Size  Field          Type         Description
---------------------------------------------------------------------------------------------
0x22    512B  i_data[4][64]  int16[4][64] In-Phase (Real) channel component per Rx & subcarrier
0x222   512B  q_data[4][64]  int16[4][64] Quadrature (Imag) channel component per Rx & subcarrier
```

Total size: $34 + 512 + 512 = 1058\text{ Bytes}$.

---

## 4. Algorithmic Deep Dive & Mathematical Bugfixes

The user identified four critical mathematical bugs in the Python DSP pipeline ([`laptop/csi_doppler.py`](file:///C:/Users/NoSet/Coding_Projects/R6800_CSI/laptop/csi_doppler.py)):

### Bug 1: Phase Sanitization AFTER Conjugate Ratio Destroys Spatial AoA

**The Bug:**
The current pipeline computes the conjugate ratio $H_0 \cdot H_1^*$, and then runs 1D linear regression on the resulting phase to subtract the slope:
$$\text{sanitized\_phase} = \text{unwrapped} - (\text{slope} \cdot k)$$

**The Flaw:**
In Wi-Fi transceivers, both antennas share the exact same Analog-to-Digital Converter (ADC) clock and RF local oscillator (LO).
The phase at subcarrier $f_k$ for antenna $i$ is:
$$\angle H_0(f_k) = \phi_0(f_k) - 2\pi f_k \tau + \theta_{\text{CFO}}$$
$$\angle H_1(f_k) = \phi_1(f_k) - 2\pi f_k \tau + \theta_{\text{CFO}}$$
where $\tau$ is the packet time-of-arrival jitter and $\theta_{\text{CFO}}$ is the Carrier Frequency Offset.

When computing the conjugate ratio:
$$\angle \left( \frac{H_0(f_k)}{H_1(f_k)} \right) = \angle H_0(f_k) - \angle H_1(f_k) = \phi_0(f_k) - \phi_1(f_k)$$
Both the timing slope $-2\pi f_k \tau$ and the oscillator drift $\theta_{\text{CFO}}$ **cancel out completely and identically**.

Any remaining phase slope across subcarriers represents the physical propagation delay difference between the two antennas in space:
$$\Delta \phi(f_k) = -2\pi f_k \frac{\Delta d}{c} = -2\pi f_k \frac{d \sin(\theta)}{c}$$
Subtracting this slope via linear regression strips out the Angle of Arrival (AoA) and spatial multipath geometry!

**Fix:**
Remove phase regression completely after computing the conjugate ratio. The conjugate ratio **is** the phase sanitization.

---

### Bug 2: Noise Explosion via Division in CSI Ratio

**The Bug:**
$$\text{CSI}_{\text{ratio}} = \frac{H_0 \cdot H_1^*}{|H_1|^2 + \epsilon}$$

**The Flaw:**
Due to destructive multipath interference, individual subcarriers on Antenna 1 frequently plunge into deep fading notches where $|H_1(f_k)|^2 \approx 0$.
Dividing by a near-zero denominator amplifies thermal noise by $40\text{ dB}$ to $60\text{ dB}$, blowing out the STFT dynamic range.

**Fix:**
Use Hermitian cross-correlation (as used in Widar 2.0 / IndoTrack):
$$C(t, f) = H_0(t, f) \cdot H_1^*(t, f)$$
If amplitude normalization is desired, normalize by the **joint total energy** of both antennas:
$$C_{\text{norm}}(t, f) = \frac{H_0(t, f) \cdot H_1^*(t, f)}{\sqrt{|H_0(t, f)|^2 + |H_1(t, f)|^2} + \epsilon}$$

---

### Bug 3: Static Clutter Filter Without Time-Normalization (Jitter Problem)

**The Bug:**
$$S(t) = \alpha X(t) + (1 - \alpha) S(t-1)$$
with a fixed scalar $\alpha = 0.04$.

**The Flaw:**
A fixed $\alpha$ assumes an strictly uniform sampling interval $\Delta t$. Over Wi-Fi, packet intervals fluctuate heavily (from $2\text{ ms}$ to $50\text{ ms}$) due to CSMA/CA contention, backoff, and packet bursts.
* During burst transmission ($\Delta t = 2\text{ ms}$), the effective cutoff frequency jumps to $f_c \approx 3.2\text{ Hz}$, stripping out slow human movements (e.g. breathing, slow walking).
* During idle periods ($\Delta t = 50\text{ ms}$), $f_c \approx 0.13\text{ Hz}$, letting massive DC wall reflection clutter leak into the Doppler spectrum.

**Fix:**
Apply one of two methods:
1. **Time-Aware Exponential Filter:** Calculate $\alpha_i$ dynamically using the packet timestamp:
   $$\alpha_i = 1 - \exp\left(-\frac{\Delta t_i}{\tau}\right) \quad \text{with } \tau = \frac{1}{2\pi f_c}, \quad \Delta t_i = \text{timestamp\_us}[i] - \text{timestamp\_us}[i-1]$$
2. **Pre-Filter Resampling:** Interpolate the non-uniform raw time series $(t_i, C(t_i, f))$ onto an exact, uniform $200\text{ Hz}$ time grid ($\Delta t = 5\text{ ms}$) using linear or cubic spline interpolation *before* feeding the clutter filter and STFT.

---

### Bug 4: Bandwidth Inconsistency (VHT80 vs. 64 Subcarriers)

**The Bug:**
The AP is configured for VHT80 (80 MHz bandwidth), but `struct mt76_csi_data` hardcodes `n_subcarriers = 64`.

**The Flaw:**
* Standard 20 MHz OFDM has an FFT size of 64 points (56 usable tones: 52 data + 4 pilots). Subcarrier spacing $\Delta f = 312.5\text{ kHz}$.
* 80 MHz VHT80 OFDM has an FFT size of 256 points (242 usable tones: 234 data + 8 pilots).
* If the hardware operates in VHT80 mode, reading only 64 subcarriers captures only the primary 20 MHz sub-channel or truncates the upper 192 tones, causing an incorrect frequency-to-distance mapping.

**Fix:**
* For 20 MHz mode: Configure router wireless to `htmode 'HT20'` or `'VHT20'`.
* For 80 MHz mode: Expand struct buffer to 256 subcarriers ($256 \times 2 \times 2 = 1024\text{ Bytes}$ per antenna).

---

### Bug 5: Radar Doppler Equation: Monostatic vs. Bistatisch

**The Bug:**
$$v = \frac{f_D \cdot \lambda}{2}$$

**The Flaw:**
This equation is only valid for **monostatic radar**, where transmitter and receiver antennas are co-located.
In our Wi-Fi sensing topology, the laptop (Transmitter) and the router (Receiver) are separated by distance $L$ (Bistatic Radar configuration).

The true bistatic Doppler shift is:
$$f_D = \frac{2v}{\lambda} \cos(\theta) \cos\left(\frac{\beta}{2}\right)$$
where:
* $\beta$ is the **bistatic angle** (angle subtended at the moving target by the transmitter and receiver).
* $\theta$ is the angle between the target velocity vector and the bistatic bisector.
* When the target is directly between transmitter and receiver ($\beta = 180^\circ$), $\cos(\beta/2) = 0$, resulting in zero Doppler shift regardless of speed!
* For relative micro-Doppler feature classification (gait recognition, gesture detection), relative frequency is sufficient, but for absolute velocity display, this angular dependency must be noted.

---

## 5. Corrected Target DSP Pipeline (Soll-Zustand)

```
[ Raw Complex CFR: H_0(t_i, f), H_1(t_i, f) ]
                    │
                    ▼
[ Time-Domain Resampling / Interpolation ]
Interpolate non-uniform packet timestamps (timestamp_us) to uniform 200 Hz grid (Δt = 5 ms)
                    │
                    ▼
[ Dual-Antenna Cross-Correlation ]
C(t, f) = H_0(t, f) · conj(H_1(t, f))
(Eliminates transceiver CFO and SFO phase jitter automatically; preserves AoA phase slope)
                    │
                    ▼
[ Dynamic Static Clutter Removal ]
Adaptive High-Pass Filter (fc ≈ 0.3 - 0.5 Hz) using time-aware EMA:
α_i = 1 - exp(-Δt_i / τ)
C_dynamic(t, f) = C(t, f) - C_static(t, f)
                    │
                    ▼
[ Subcarrier Selection / Aggregation ]
Option A: Principal Component Analysis (PCA) - extract 1st principal component
Option B: Variance-weighted aggregation across sensitive subcarriers
                    │
                    ▼
[ Short-Time Fourier Transform (STFT) ]
Sliding Hanning Window (N = 256, 1.28 s, 85% overlap), 512-point FFT
                    │
                    ▼
[ Micro-Doppler Time-Velocity Spectrogram ]
Logarithmic Power Spectrum (dB), Velocity grid mapped via bistatic wavelength
```

---

## 6. Actionable Next Steps for Future Agents / Developers

### Phase A: Fix the Laptop Signal Processing (Immediate)
1. **Edit [`laptop/csi_doppler.py`](file:///C:/Users/NoSet/Coding_Projects/R6800_CSI/laptop/csi_doppler.py):**
   * Replace the division-based conjugate ratio with cross-correlation:
     ```python
     c = h0 * np.conj(h1)
     norm = np.sqrt(np.abs(h0)**2 + np.abs(h1)**2) + 1e-6
     csi_corr = c / norm
     ```
   * Remove the phase sanitization call `_sanitize_phase_1d()` from `csi_corr`.
   * Implement time-aware dynamic alpha in `StaticClutterFilter` based on `dt = (t_now - t_last)`.
   * Add uniform grid resampling (200 Hz) before the STFT buffer.

### Phase B: Reconcile Channel Bandwidth
1. Set OpenWrt wireless configuration to fixed 20 MHz to match 64 subcarriers:
   ```bash
   uci set wireless.radio1.htmode='HT20'
   uci commit wireless
   wifi reload
   ```
   Or expand `mt76_csi.h` and the extractor to support 256 subcarriers for VHT80.

### Phase C: Solve the Hardware CFR Extraction on MT7615
1. **Investigate Andes N9 Firmware Beamforming Feedback:**
   * Inspect OpenWrt kernel tree: `mt76/mt7615/mcu.c` and `mt76/mt7615/mac.c`.
   * Search for VHT Compressed Beamforming Report handling (`ACTION_VHT_COMPRESSED_BF`).
   * Test whether sending 802.11ac NDP sounding frames from the laptop triggers firmware beamforming matrices.
2. **Analyze MT7615 Baseband Testmode / DMA Capture:**
   * Inspect `mt7615_mac_fill_tm_rx()` in `mt7615/mac.c` to see how MediaTek's testmode dumps raw I/Q samples.

---

## 7. Developer Cheat-Sheet & Verified Commands

### Router PuTTY Serial Console (115200 baud):
```bash
# Check status of radios and AP
iwinfo
ip addr show br-lan

# Restart networking / Wi-Fi
wifi down
wifi up

# Hot-reload custom mt76 kernel modules
killall csi_extractor 2>/dev/null
wifi down
rmmod mt7615e 2>/dev/null; rmmod mt7615_common 2>/dev/null; rmmod mt7603e 2>/dev/null; rmmod mt76_connac_lib 2>/dev/null; rmmod mt76 2>/dev/null

insmod /tmp/mt76.ko
insmod /tmp/mt76-connac-lib.ko
insmod /tmp/mt7603e.ko
insmod /tmp/mt7615-common.ko
insmod /tmp/mt7615e.ko
wifi up

# Run CSI Extractor Daemon
PHY=$(ls /sys/kernel/debug/ieee80211/ | grep phy | tail -1)
/tmp/csi_extractor -i $PHY -d 192.168.10.50 -p 5500 -e
```

### Windows Host (PowerShell):
```powershell
# Copy fresh modules to router over Wi-Fi
scp -O C:\Users\NoSet\Coding_Projects\R6800_CSI\deploy\*.ko root@192.168.10.1:/tmp/

# High-Rate Traffic Generator (200 Hz UDP burst loop)
$u = New-Object System.Net.Sockets.UdpClient; $ep = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Parse("192.168.10.1"), 9999); $b = [byte[]](1..64); while($true){ [void]$u.Send($b, $b.Length, $ep); Start-Sleep -Milliseconds 5 }

# Run Visualizer
python C:\Users\NoSet\Coding_Projects\R6800_CSI\laptop\csi_doppler.py --port 5500
```

### WSL Ubuntu-24.04 (Build Machine):
```bash
# Regenerate all patches with 100% diff accuracy
python3 /mnt/c/Users/NoSet/.gemini/antigravity-cli/brain/3df0222c-5dc1-45fd-b69c-10ad9182e0a6/scratch/create_patches.py

# Recompile mt76 modules
bash /mnt/c/Users/NoSet/Coding_Projects/R6800_CSI/build/build.sh mt76
```
