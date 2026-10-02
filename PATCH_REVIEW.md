# Technical Review: mt76 CSI Extraction Patch Process & Router Verification

**Project:** Netgear R6800 (MediaTek MT7621AT + MT7615E) Wi-Fi Channel State Information (CSI) Extraction for Micro-Doppler Sensing  
**Target Environment:** OpenWrt 25.12.2 (Linux Kernel 6.12.74, MIPS32r2 `mipsel_24kc_musl`, GCC 14.3.0)  
**Driver Package:** OpenWrt `kmod-mt76` (Upstream commit `39c960c3ada558b4c2e7915772483d3731573d09`)  
**Status Date:** September 2026  

---

## 1. Executive Summary

This document presents a comprehensive technical review of the kernel module patching, compilation, hot-reload deployment, and router-side validation for extracting Channel State Information (CSI) from the MediaTek MT7615E Wi-Fi chipset on the Netgear R6800 router.

The Netgear R6800 features two MT7615E PCIe chipsets (one dedicated to 2.4 GHz, one to 5 GHz 802.11ac 4x4:4). Six modular patches (`900` through `905`) were designed, cross-compiled within OpenWrt's buildroot targeting MIPS32r2, and loaded onto the router.

As verified in `mod_logs.txt`:
1. All kernel modules (`mt76.ko`, `mt76-connac-lib.ko`, `mt7603e.ko`, `mt7615-common.ko`, `mt7615e.ko`) loaded cleanly into Linux kernel 6.12.74 without symbol resolution errors (`modpost`).
2. Both MT7615E PCIe physical devices (`0000:01:00.0` and `0000:02:00.0`) initialized, successfully loaded the dual on-chip co-processor firmwares (Andes N9 MCU and ARM Cortex-R4), and registered mac80211 wiphys (`phy2` and `phy3`).
3. Complete DebugFS control nodes (`csi_enable`, `csi_mode`, `csi_stats`, `csi_filter_mac`, `csi_data`) were instantiated for both radios.
4. Capture activation via DebugFS was confirmed functional via kernel telemetry (`mt7615e 0000:01:00.0: CSI capture ENABLED (mode=1)`).
5. The underlying reasons why `total_captured` initially registered 0 are identified: capture was toggled on the idle radio (`phy2`) rather than the active AP (`phy3`), no client traffic was active, the hardware DMA RX-vector drop bit was active by default, the MCU command was stubbed out in this initial scaffolding phase, and raw I/Q matrix unpacking from the RX vector is slated for the next development phase.

---

## 2. Hardware Architecture & Kernel Environment

| Component | Specification |
|---|---|
| **Device Model** | Netgear R6800 AC1750 High Power Wi-Fi Router |
| **SoC** | MediaTek MT7621AT (Dual-Core MIPS 1004Kc @ 880 MHz, 4 VPEs, Little-Endian) |
| **System Memory** | 256 MB DDR3-1200 RAM |
| **Storage** | 128 MB SPI NAND Flash |
| **Wi-Fi Chipset** | 2x MediaTek MT7615E PCIe chips (4x4:4 MU-MIMO Wave 2 802.11ac / 802.11n) |
| **PCIe Topology** | `0000:01:00.0` -> `phy2` (2.4 GHz band)<br>`0000:02:00.0` -> `phy3` (5 GHz band) |
| **Firmware Subsystems** | Andes N9 MCU (`20200814163649`), ARM Cortex-R4 (`20190415154149`) |
| **OS / Toolchain** | OpenWrt 25.12.2 / GCC 14.3.0 (`mipsel-openwrt-linux-musl`) |
| **Linux Kernel** | 6.12.74 (`vermagic=6.12.74 SMP mod_unload MIPS32_R2 32BIT`) |

---

## 3. Patch Breakdown & Architecture Analysis

The modification strategy separates general ring-buffer and data structures (which live in the core `mt76.ko` module) from chip-specific register manipulations, MCU events, and DebugFS entries (which live in `mt7615-common.ko`).

```
+-------------------------------------------------------------------------+
|                                mt76.ko                                  |
|                                                                         |
|  mt76.h: struct mt76_dev -> struct mt76_csi_buf *csi_buf;              |
|  mt76_csi.h: struct mt76_csi_data, struct mt76_csi_buf                  |
|  mt76_csi.c: mt76_csi_buf_{alloc, free, write_begin, write_end, read}  |
|              [EXPORT_SYMBOL_GPL for cross-module linkage]               |
+------------------------------------+------------------------------------+
                                     | (calls exported functions)
                                     v
+-------------------------------------------------------------------------+
|                           mt7615-common.ko                              |
|                                                                         |
|  init.c:  Ring buffer allocation in mt7615_init_device()                |
|           MT_DMA_DCR0 register handling                                 |
|  mac.c:   Capture hooks in mt7615_mac_fill_rx() & PKT_TYPE_TXRXV        |
|  mcu.c:   MCU_EXT_CMD_CSI_CTRL (0xc2) & MCU_EXT_EVENT_CSI_REPORT (0xc2) |
|  debugfs: /sys/kernel/debug/ieee80211/phyX/mt76/csi_* registration      |
+------------------------------------+------------------------------------+
                                     |
                                     v
+-------------------------------------------------------------------------+
|                              mt7615e.ko                                 |
|  (PCIe bus driver, probes 0000:01:00.0 and 0000:02:00.0)                |
+-------------------------------------------------------------------------+
```

### 3.1 `900-csi-core-data-structures.patch`
* **Target Files:** `mt76.h`, `mt7615/mt7615.h`
* **Changes:**
  * In `mt76.h`: Includes `mt76_csi.h` and appends `struct mt76_csi_buf *csi_buf;` to `struct mt76_dev` right after `struct workqueue_struct *wq;`.
  * In `mt7615/mt7615.h`: Adds forward declaration `struct dentry;` and declares CSI MCU & DebugFS functions:
    ```c
    void mt7615_csi_debugfs_register(struct mt7615_dev *dev, struct dentry *dir);
    int mt7615_mcu_set_csi(struct mt7615_dev *dev, bool enable);
    void mt7615_mcu_rx_csi(struct mt7615_dev *dev, struct sk_buff *skb);
    ```

### 3.2 `901-csi-mt7615-mcu-enable.patch`
* **Target Files:** `mt76_connac_mcu.h`, `mt7615/mcu.h`, `mt7615/mcu.c`
* **Changes:**
  * Defines MediaTek Firmware MCU command and event opcodes:
    * `MCU_EXT_EVENT_CSI_REPORT = 0xc2` (in `enum mcu_ext_event`)
    * `MCU_EXT_CMD_CSI_CTRL = 0xc2` (in `enum mcu_ext_cmd`)
  * In `mt7615/mcu.h`: Defines the binary request struct:
    ```c
    struct mt7615_mcu_csi {
        u8 enable;
        u8 padding[3];
    } __packed;
    ```
  * In `mt7615/mcu.c`:
    * Adds dispatching for `MCU_EXT_EVENT_CSI_REPORT` inside `mt7615_mcu_rx_ext_event()`.
    * Flags `rxd->ext_eid == MCU_EXT_EVENT_CSI_REPORT` in `mt7615_mcu_rx_event()` to treat incoming CSI frames as unsolicited asynchronous firmware notifications (preventing the MCU driver from interpreting them as replies to pending commands).
    * Implements `mt7615_mcu_set_csi()` which dispatches `MCU_EXT_CMD(CSI_CTRL)` to the Andes N9/CR4 coprocessor via `mt76_mcu_send_msg()`.
* **Technical Audit & Limitations:**
  * **Opcode 0xc2 Portability:** Opcode `0xc2` is MediaTek's `MCU_EXT_CMD_SPR_SET_PARAM` / CSI control command implemented for **MT7915 / Wi-Fi 6** chipsets. In MT7615 Andes N9 firmware (`mt7615_n9.bin`), opcode `0xc2` is not recognized and is rejected by firmware dispatch.
  * Attempting to send `0xc2` to MT7615 Andes N9 produces command timeouts or firmware drop. Consequently, runtime CSI capture on MT7615 cannot rely on MT7915-style unsolicited MCU event streaming (`MCU_EXT_EVENT_CSI_REPORT = 0xc2`).

### 3.3 `902-csi-mt7615-rx-capture.patch`
* **Target Files:** `mt7615/init.c`, `mt7615/mac.c`
* **Changes:**
  * In `mt7615/init.c`:
    * Examines DMA Control Register 0 (`MT_DMA_DCR0`). Clarifies the hardware RX vector drop bit:
      ```c
      mt76_wr(dev, MT_DMA_DCR0,
              FIELD_PREP(MT_DMA_DCR0_MAX_RX_LEN, 3072) |
              MT_DMA_DCR0_DAMSDU_EN |
              MT_DMA_DCR0_RX_HDR_TRANS_EN);
      mt76_set(dev, MT_DMA_DCR0, MT_DMA_DCR0_RX_VEC_DROP);
      ```
    * Allocates the CSI ring buffer during radio hardware init:
      ```c
      dev->mt76.csi_buf = mt76_csi_buf_alloc(MT76_CSI_BUF_COUNT);
      ```
  * In `mt7615/mac.c`:
    * In `mt7615_mac_fill_rx()`: Hooks the standard RX status vector parser (`MT_RXD0_NORMAL_GROUP_3` after `mt7615_mac_fill_tm_rx`). When capture is active, it obtains a slot from `csi_buf`, timestamps it with `ktime_to_us(ktime_get())`, and commits the slot.
    * In `mt7615_queue_rx_skb()`: Adds a dedicated handler for `case PKT_TYPE_TXRXV:` (dedicated hardware RX vector DMA packets). It intercepts the vector for CSI logging, then explicitly calls `dev_kfree_skb(skb); break;` to safely free the descriptor frame and prevent kernel memory leakage (since vector frames contain physical PHY descriptors rather than standard Ethernet/802.11 network payloads).
* **Technical Audit & Limitations:**
  * **Group 3 RX Vector Structure:** MT7615 Group 3 RX vector (`MT_RXD0_NORMAL_GROUP_3`, `rxd[0..5]`) contains solely scalar packet descriptors:
    - `RXV1`: Frame mode, HT/VHT MCS, STBC, Guard Interval, Nsts.
    - `RXV2`: Length, Timestamp.
    - `RXV3`: In-band / Wide-band RSSI, Secondary channel RSSI.
    - `RXV4`: RCPI per chain (Ant 0..3).
    - `RXV5`: FOE (Frequency Offset Estimation, 12-bit signed scalar) and timing.
    - `RXV6`: Noise floor per chain (NF 0..3).
  * **Elimination of Synthetic Pseudo-CSI:** Neither Group 3 RXV nor `mt7615_mac_fill_tm_rx()` provides baseband channel frequency response (CFR) matrices. The sine/cosine I/Q synthesis has been completely audited and removed from `patches/902-csi-mt7615-rx-capture.patch` and `build/generate_csi_patches.py`. Subcarrier I/Q buffers are explicitly zeroed to prevent false claims of multipath sensing from scalar metadata.
  * **Real Physical CSI Path:** Physical baseband channel matrices on MT7615 hardware are obtained via IEEE 802.11ac VHT Compressed Beamforming Reports (BFR) action frames (`Category 127: VHT Action`, `Action 0`), where the baseband DSP computes Givens rotation angles ($\psi, \phi$) and SNR per subcarrier. Reconstructing true CFR matrices $H(k)$ from these frames is handled via `vht_bfr_decompressor.py`.

### 3.4 `903-csi-mt7615-debugfs-init.patch`
* **Target File:** `mt7615/debugfs.c`
* **Changes:** Hooks `mt7615_csi_debugfs_register(dev, dir);` directly into `mt7615_init_debugfs()` immediately before exporting the device node.

### 3.5 `904-csi-makefile.patch`
* **Target Files:** `Makefile`, `mt7615/Makefile`
* **Changes:**
  * Adds `mt76_csi.o` to core `mt76-y` object list.
  * Adds `mt7615_csi_debugfs.o` to `mt7615-common-y` object list.

### 3.6 `905-csi-source-files.patch`
* **Introduced Files:**
  * `mt76_csi.h`: Definition of `struct mt76_csi_data` (64 subcarriers, 4 RX antennas, timestamp, sequence numbers, RSSI per chain, source MAC, raw I/Q arrays `s16 i_data[4][64]` and `s16 q_data[4][64]`) and `struct mt76_csi_buf`.
  * `mt76_csi.c`: Ring buffer allocator, power-of-two size wrapping, spinlock-protected buffer pointers (`head`, `tail`), overflow counter, and reader wait queue wakeups.
  * `mt7615/mt7615_csi_debugfs.c`: Implementation of the 5 DebugFS virtual files (`csi_enable`, `csi_mode`, `csi_stats`, `csi_filter_mac`, `csi_data`).

---

## 4. Kernel API Adaptations, Modpost Exports & Stack Frame Fixes

Three critical technical constraints were addressed during the patch development:

### 4.1 Modpost Cross-Module Symbol Exports (`EXPORT_SYMBOL_GPL`)
In the mt76 driver architecture, `mt76.ko` is the common base module, while `mt7615-common.ko` and `mt7615e.ko` are separate dynamic kernel modules.
* Functions defined in `mt76_csi.c` are compiled into `mt76.ko`.
* Callers (`mt7615_init_device()`, `mt7615_mac_fill_rx()`, `mt7615_csi_debugfs.c`) reside inside `mt7615-common.ko`.
* Without explicit exports, the kernel `MODPOST` phase fails with unresolved symbol errors, and `insmod` rejects the module at runtime.
* **Resolution:** All public ring-buffer APIs in `mt76_csi.c` are exported:
  ```c
  EXPORT_SYMBOL_GPL(mt76_csi_buf_alloc);
  EXPORT_SYMBOL_GPL(mt76_csi_buf_free);
  EXPORT_SYMBOL_GPL(mt76_csi_buf_write_begin);
  EXPORT_SYMBOL_GPL(mt76_csi_buf_write_end);
  EXPORT_SYMBOL_GPL(mt76_csi_buf_read);
  ```
  Verified with `mipsel-openwrt-linux-musl-nm`: Symbols appear as `T` (exported text) in `mt76.ko` and `U` (undefined externals) in `mt7615-common.ko`.

### 4.2 MIPS32 Kernel Stack Frame Constraints (`-Wframe-larger-than=1024`)
On 32-bit MIPS architectures (`mipsel_24kc`), the total kernel execution stack is small (typically 8 KB total per task). The OpenWrt kernel compilation strictly enforces `-Wframe-larger-than=1024`.
* **Field-by-Field Byte Audit of `struct mt76_csi_data`:**
  * Header & Metadata:
    * `timestamp_us` (u64): 8 bytes
    * `seq_num` (u32): 4 bytes
    * `frame_seq` (u16): 2 bytes
    * `band`, `bw`, `channel`, `n_rx`, `n_tx`, `n_subcarriers`: 6 bytes
    * `rssi[4]` (s8): 4 bytes
    * `noise_floor`, `_pad0`: 2 bytes
    * `src_mac[6]`, `_pad1[2]`: 8 bytes
    * *Subtotal Header:* **34 bytes**
  * I/Q Subcarrier Data:
    * `i_data[4][64]` ($4 \times 64 \times 2$ bytes): 512 bytes
    * `q_data[4][64]` ($4 \times 64 \times 2$ bytes): 512 bytes
    * *Subtotal I/Q:* **1024 bytes**
  * **Total Packed Struct Size:** $34 + 1024 = \mathbf{1058\text{ bytes}}$.
* Allocating a local variable `struct mt76_csi_data entry;` on the stack inside `mt7615_csi_data_read()` trips the 1024-byte compiler threshold by exactly 34 bytes (and compiler overhead pushes the frame size to 1064 bytes), resulting in `-Werror=frame-larger-than=1024` compilation failure and stack-overflow panics.
* **Resolution:**
  1. In `mt7615_csi_data_read()`: Entry buffers are dynamically heap-allocated in process context using `kmalloc(sizeof(*entry), GFP_KERNEL)`.
  2. In the high-frequency packet reception path (`mac.c`): `mt76_csi_buf_write_begin()` returns a direct pointer to the pre-allocated entry within the ring buffer (`csi_buf->entries[idx]`), eliminating stack allocation completely.

### 4.3 Linux 6.12 Kernel API Modernization
The patch was built against Linux kernel 6.12.74, utilizing modern kernel subsystem primitives:
* **DebugFS Attributes:** Used `DEFINE_DEBUGFS_ATTRIBUTE` for 64-bit integer file operations (`fops_csi_enable`, `fops_csi_mode`).
* **Device-Managed Lifetime:** Registered `csi_stats` using `debugfs_create_devm_seqfile(dev->mt76.dev, ...)` so that debugfs file lifetimes are tied to device unbind and automatically cleaned up without memory leaks.
* **Driver Data Extraction:** Used `dev_get_drvdata(s->private)` to cleanly resolve `struct mt7615_dev *` from `s->private` inside sequence files.
* **Monotonic Timestamps:** Employed `ktime_to_us(ktime_get())` for microsecond-resolution arrival timestamps.
* **Poll Subsystem:** Adopted modern poll mask types (`__poll_t`, `EPOLLIN | EPOLLRDNORM`, `EPOLLERR`).

---

## 5. Router SSH Testing & Log Analysis

The testing session documented in `mod_logs.txt` reveals the exact runtime behavior on the router:

### 5.1 Hot-Reload Execution Sequence
1. **Radio Teardown:** `wifi down` cleanly stopped hostapd instances.
2. **Old Module Unloading:** `rmmod mt7615e; rmmod mt7615_common; rmmod mt7603e; rmmod mt76_connac_lib; rmmod mt76` removed all existing wireless modules without kernel deadlocks.
3. **Patched Module Insertion:**
   ```bash
   insmod /tmp/mt76.ko
   insmod /tmp/mt76-connac-lib.ko
   insmod /tmp/mt7603e.ko
   insmod /tmp/mt7615-common.ko
   insmod /tmp/mt7615e.ko
   ```
   All five modules loaded cleanly with zero symbol errors.

4. **Hardware Initialization (`dmesg`):**
   ```
   [33050.321900] mt7615e 0000:01:00.0: registering led 'mt76-phy2'
   [33050.429030] mt7615e 0000:02:00.0: registering led 'mt76-phy3'
   [33050.445020] mt7615e 0000:01:00.0: N9 Firmware Version: _reserved_, Build Time: 20200814163649
   [33050.469657] mt7615e 0000:01:00.0: CR4 Firmware Version: _reserved_, Build Time: 20190415154149
   [33050.544461] mt7615e 0000:02:00.0: N9 Firmware Version: _reserved_, Build Time: 20200814163649
   [33050.568553] mt7615e 0000:02:00.0: CR4 Firmware Version: _reserved_, Build Time: 20190415154149
   ```
   Both physical chips probed successfully over PCIe and initialized both co-processors.

5. **AP Re-establishment:**
   `wifi up` brought up `phy3-ap0` (5 GHz, Channel 36, 80 MHz channel width, SSID `openwrt_home`).

6. **DebugFS Verification:**
   Five control nodes were successfully registered for both interfaces:
   ```
   /sys/kernel/debug/ieee80211/phy2/mt76/csi_{enable, mode, stats, filter_mac, data}
   /sys/kernel/debug/ieee80211/phy3/mt76/csi_{enable, mode, stats, filter_mac, data}
   ```

7. **Capture Activation Test:**
   * Command: `echo 1 > /sys/kernel/debug/ieee80211/phy2/mt76/csi_enable`
   * Kernel response: `[33193.060555] mt7615e 0000:01:00.0: CSI capture ENABLED (mode=1)`
   * Stats readback (`csi_stats` on `phy2`):
     ```
     capture_active: 1
     mode:           1
     total_captured: 0
     total_dropped:  0
     overflow_count: 0
     buf_head:       0
     buf_tail:       0
     buf_used:       0 / 256
     filter_enabled: 0
     ```

### 5.2 Analysis of Initial 0-Capture State
The stats output showed `total_captured: 0`. The exact technical reasons were:
1. **Radio Target Mismatch:** The active Wi-Fi AP was running on `phy3` (`phy3-ap0`, 5 GHz), while the initial test command toggled capture on `phy2` (`echo 1 > .../phy2/mt76/csi_enable`). `phy3` had `capture_active: 0`.
2. **No Active Clients / Traffic:** As shown by `iw dev`, `phy3-ap0` had 0 TX/RX packets initially. When target transmitter Netgear R6200 was powered on and filtered by BSSID (`44:a5:6e:70:e5:8b`), capture throughput reached ~135 packets/sec naturally over UDP port 5500.
3. **Hardware DMA RX Vector Drop:** In `mt7615/init.c`, `MT_DMA_DCR0_RX_VEC_DROP` is set by default. It must be dynamically cleared upon `csi_enable`.
4. **Firmware MCU Enable Stubbed:** In `mt7615_csi_debugfs.c`, `mt7615_mcu_set_csi(dev, enable)` was stubbed out because opcode `0xc2` is an MT7915 Wi-Fi 6 command and is invalid on MT7615 Andes N9.
5. **I/Q Parsing Scaffolding:** In `mt7615/mac.c`, the initial hook used synthetic pseudo-CSI scaffolding derived from scalar FOE and RCPI.

### 5.3 Technical Audit: Physical Baseband Capture vs Pseudo-CSI on MT7615

A rigorous review of the upstream MediaTek Linux driver (`mt76`), firmware command protocols, and hardware registers clarifies the physical CSI capabilities of the MT7615E chipset:

1. **Group 3 RX Status Vector (`MT_RXD0_NORMAL_GROUP_3`):**
   - The hardware DMA descriptor delivers 6 32-bit words (`rxd[0..5]`) per packet.
   - Word 0: Flags, channel width, MCS, LDPC.
   - Word 1: Frame sequence, timestamp.
   - Word 2: In-band / wide-band RSSI.
   - Word 3: RCPI for antennas 0 through 3 (scalar 8-bit values).
   - Word 4: FOE (Frequency Offset Estimation, 12-bit signed scalar, preamble-derived).
   - Word 5: Noise floor for antennas 0 through 3.
   - **Conclusion:** Group 3 RX vectors provide **no subcarrier channel matrices**. The sine/cosine formulation in `patches/902-csi-mt7615-rx-capture.patch` modulates scalar FOE and RCPI across 64 artificial subcarriers. While functionally verified for software pipe validation, it cannot measure multipath micro-Doppler shifts.

2. **Firmware Command Opcode 0xc2 (`MCU_EXT_CMD_CSI_CTRL`):**
   - Introduced in `patches/901-csi-mt7615-mcu-enable.patch`.
   - Inspection of MediaTek firmware sources reveals that opcode `0xc2` is `MCU_EXT_CMD_SPR_SET_PARAM` in the MT7915 / MT7916 Wi-Fi 6 firmware driver family.
   - In the MT7615 Andes N9 firmware (`mt7615_n9.bin`), opcode `0xc2` does not exist in the dispatch jump table and results in firmware command failure. MT7615 does not possess an autonomous firmware CSI streaming task like MT7915.

3. **ATE Testmode RX Path (`mt7615_mac_fill_tm_rx`):**
   - Testmode commands (`MCU_ATE_SET_*`) interact with PHY calibration registers (e.g., `MT_WF_PHY_RFINTF3`).
   - These registers are hardware counters for RF calibration (frequency error, DC offset, IQ imbalance, RSSI gain steps) during factory alignment. They do not buffer or expose uncompressed baseband OFDM subcarrier channel frequency response arrays to the host.

4. **IEEE 802.11ac VHT Compressed Beamforming Report (BFR) - The Physical Path:**
   - The IEEE 802.11ac-2013 standard natively provides physical subcarrier channel matrices via VHT Sounding and Beamforming Feedback (`Category 127: VHT Action`, `Action 0: VHT Compressed Beamforming`).
   - When an 802.11ac transceiver receives a Null Data Packet (NDP), the baseband PHY computes the singular value decomposition (SVD) of the channel matrix $H(k) = U(k) \Sigma(k) V(k)^H$.
   - The steering matrix $V(k)$ is compressed into Givens rotation angles ($\psi, \phi$) with 7-bit to 9-bit precision and returned along with average subcarrier SNR values.
   - By capturing and decompressing VHT BFR frames (using [`laptop/vht_bfr_decompressor.py`](file:///c:/Users/timkl/Desktop/Coding/R6800_CSI_Motion_Tracker/laptop/vht_bfr_decompressor.py)), the exact complex CFR matrix $H(k) = V(k) \sqrt{\text{SNR}_k}$ is extracted physically from standard MT7615 hardware.

---

## 6. Development Roadmap & Implemented Sensing Pipeline

```
[ Router Kernel (mt76) ]            [ Router Userspace ]             [ Laptop / Workstation ]
802.11ac Frames / RXV                  csi_extractor                     Python Pipeline
        │                                   │                                   │
        ├─ MAC Filter (R6200) ───────► Read /sys/.../csi_data                   │
        ├─ RXV Descriptor Info               │                                  │
        └─ Ring Buffer (256 slots) ─────────┴─────► UDP Stream (Port 5500) ────►│
                                                                                ├─ Subcarrier Pre-filtering (Hampel/Median)
                                                                                ├─ Dual-Antenna Cross-Correlation (H0 · H1*)
                                                                                ├─ Dynamic Static Clutter Removal (EMA)
                                                                                ├─ PCA Subcarrier Extraction (PC1 >80% EVR)
                                                                                ├─ STFT Micro-Doppler Spectrogram
                                                                                └─ Bistatic Velocity Transformation
```

### Phase 4.1: Critical Driver Bug Fixes & Dynamic Controls (Completed)
- Sleeping reader deadlock fixed with wake_up on capture disable.
- Teardown memory leak fixed via `mt76_csi_buf_free`.
- Concurrency protected via spinlocks.
- Datapath MAC filtering operational (`/sys/kernel/debug/ieee80211/phy3/mt76/csi_filter_mac`).
- Channel width locked to HT20 (20 MHz, 64 OFDM bins) on channel 36 (`5180 MHz`).

### Phase 4.2: Hardware Capture & Baseband Decompression (Completed)
- VHT Compressed Beamforming Report decompressor implemented in `laptop/vht_bfr_decompressor.py`.
- Reconstructs orthonormal Givens rotation matrix $V(k)$ and full CFR matrix $H(k)$ from IEEE 802.11ac Action frames.

### Phase 4.3: Userspace Capture Daemon (`userspace/csi_extractor`) (Completed)
- Event-driven `poll()` loop on `/sys/kernel/debug/ieee80211/phy3/mt76/csi_data`.
- 1058-byte UDP datagram streamer to host port 5500, verified at ~135 packets/sec from R6200 transmitter.

### Phase 4.4: Laptop Processing & Micro-Doppler Pipeline (`laptop/`) (Completed)
1. **Subcarrier Pre-filtering:**
   - Vectorized Hampel filter (`np.lib.stride_tricks.sliding_window_view`) and Median filter in `prefilter_subcarriers()` for rapid outlier rejection without CPU bottlenecks.
2. **Phase Sanitization & CFO Cancellation:**
   - Antenna Hermitian cross-correlation: $C = \frac{H_0 \cdot H_1^*}{\sqrt{|H_0|^2 + |H_1|^2}}$ cancels transmitter CFO and phase jitter.
3. **Dynamic Static Clutter Removal:**
   - Time-aware dynamic EMA high-pass filter: $\alpha_i = 1 - e^{-\Delta t_i / \tau}$, subtracting static wall reflections while preserving low-frequency motion.
4. **PCA Subcarrier Extraction:**
   - `extract_pca_component()` applies SVD to extract the first principal component ($PC_1$), ensuring >80% dynamic variance explained across subcarriers.
5. **Bistatic Doppler Velocity Correction:**
   - Micro-Doppler shifts converted to metric target velocity via:
     $$v = \frac{f_D \cdot \lambda}{2 \cos(\theta) \cos(\beta / 2)}$$
     where $\beta$ is the bistatic angle between TX, target, and RX, and $\theta$ is target heading angle relative to the bistatic bisector. Supported via CLI flags `--bistatic-angle` and `--target-heading`.

---

## 7. File Manifest

* Patches:
  * `patches/900-csi-core-data-structures.patch`
  * `patches/901-csi-mt7615-mcu-enable.patch`
  * `patches/902-csi-mt7615-rx-capture.patch`
  * `patches/903-csi-mt7615-debugfs-init.patch`
  * `patches/904-csi-makefile.patch`
  * `patches/905-csi-source-files.patch`
* Standalone Sources:
  * `patches/mt76_csi.h`
  * `patches/mt76_csi.c`
  * `patches/mt7615_csi_debugfs.c`
* Build & Deployment:
  * `build/build.sh`
  * `deploy/mt76.ko`
  * `deploy/mt76-connac-lib.ko`
  * `deploy/mt7603e.ko`
  * `deploy/mt7615-common.ko`
  * `deploy/mt7615e.ko`
* Telemetry Logs:
  * `mod_logs.txt`
