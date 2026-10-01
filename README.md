# R6800 CSI Extraction Project
# Netgear R6800 (MT7621AT + MT7615E) - WiFi CSI for Micro-Doppler Analysis

## Hardware
- **SoC:** MediaTek MT7621AT (Dual-Core MIPS 1004Kc, 880MHz)
- **WLAN:** MediaTek MT7615E (4x4 dual-band, 802.11ac)
- **RAM:** 256MB DDR3
- **Flash:** 128MB NAND
- **Firmware:** OpenWrt 25.12.2

## Project Structure
```
R6800_CSI/
├── patches/          # OpenWrt mt76 kernel patches (quilt format)
├── userspace/        # CSI extractor tool (runs on router)
├── laptop/           # Python receiver & micro-Doppler analysis
├── build/            # Build helper scripts & Makefile
├── deploy/           # Compiled .ko modules for quick deployment
└── docs/             # Hardware documentation, pinouts, notes
```

## Quick Commands
```bash
# Build mt76 module only
wsl -- bash -c "cd ~/openwrt && make package/kernel/mt76/compile V=s"

# Deploy to router
scp deploy/*.ko root@<router-ip>:/tmp/

# Hot-reload on router
ssh root@<router-ip> 'wifi down; rmmod mt7615e; rmmod mt76; insmod /tmp/mt76.ko; insmod /tmp/mt7615e.ko; wifi up'

# Enable CSI capture
ssh root@<router-ip> 'echo 1 > /sys/kernel/debug/ieee80211/phy0/mt76/csi_enable'
```
