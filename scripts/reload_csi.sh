#!/bin/sh
set -e

echo "[+] Stopping WiFi..."
wifi down || true
sleep 2

echo "[+] Unloading existing mt76 modules..."
rmmod mt7615e || true
rmmod mt7615_common || true
rmmod mt7603e || true
rmmod mt76_connac_lib || true
rmmod mt76 || true
sleep 1

echo "[+] Loading custom CSI-enabled mt76 modules..."
insmod /tmp/mt76.ko
insmod /tmp/mt76-connac-lib.ko
insmod /tmp/mt7603e.ko
insmod /tmp/mt7615-common.ko
insmod /tmp/mt7615e.ko
sleep 2

echo "[+] Restarting WiFi..."
wifi up
sleep 3

echo "[+] Loaded mt76 modules:"
lsmod | grep mt76

echo "[+] Checking CSI debugfs nodes:"
ls -la /sys/kernel/debug/ieee80211/phy*/mt76/csi_* 2>/dev/null || echo "[!] DebugFS csi nodes not found yet"

echo "[+] Done."
