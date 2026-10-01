#!/bin/bash
# ============================================================
# Phase 2 - Schritt 2: OpenWrt Build-System Setup
# In WSL2 (Ubuntu) ausfuehren:
#   bash /mnt/c/Users/NoSet/Coding_Projects/R6800_CSI/phase2_openwrt_setup.sh
# ============================================================

set -e
echo "=== R6800 CSI Projekt - OpenWrt Build-System Setup ==="
echo "Ziel: OpenWrt 25.12.2 fuer Netgear R6800 (ramips/mt7621)"
echo ""

# ----------------------------------------
# 1. Build-Dependencies installieren
# ----------------------------------------
echo "[1/7] Installiere Build-Dependencies..."
sudo apt update
sudo apt install -y \
    build-essential clang flex bison g++ gawk \
    gcc-multilib g++-multilib gettext git libncurses5-dev \
    libssl-dev python3-distutils python3-setuptools python3-dev \
    rsync swig unzip zlib1g-dev file wget curl \
    qemu-utils libelf-dev autoconf automake libtool \
    python3-pyelftools subversion

echo "[1/7] Dependencies installiert. OK"

# ----------------------------------------
# 2. OpenWrt Source klonen
# ----------------------------------------
OPENWRT_DIR="$HOME/openwrt"

if [ -d "$OPENWRT_DIR" ]; then
    echo "[2/7] OpenWrt-Verzeichnis existiert bereits: $OPENWRT_DIR"
    echo "      Ueberspringe Klonen."
else
    echo "[2/7] Klone OpenWrt 25.12.2..."
    git clone https://git.openwrt.org/openwrt/openwrt.git "$OPENWRT_DIR"
    cd "$OPENWRT_DIR"
    git checkout v25.12.2
    echo "[2/7] OpenWrt v25.12.2 geklont. OK"
fi

cd "$OPENWRT_DIR"

# ----------------------------------------
# 3. Feeds aktualisieren
# ----------------------------------------
echo "[3/7] Aktualisiere Feeds..."
./scripts/feeds update -a
./scripts/feeds install -a
echo "[3/7] Feeds installiert. OK"

# ----------------------------------------
# 4. config.buildinfo holen fuer exaktes Vermagic-Match
# ----------------------------------------
echo "[4/7] Lade config.buildinfo fuer ramips/mt7621..."
CONFIG_URL="https://downloads.openwrt.org/releases/25.12.2/targets/ramips/mt7621/config.buildinfo"
wget -O .config "$CONFIG_URL" 2>/dev/null || {
    echo "WARNUNG: config.buildinfo Download fehlgeschlagen."
    echo "Fallback: Manuelle Konfiguration noetig (make menuconfig)"
    echo "  Target System: MediaTek Ralink MIPS"
    echo "  Subtarget: MT7621 based boards"
    echo "  Target Profile: Netgear R6800"
}
echo "[4/7] Konfiguration geladen. OK"

# ----------------------------------------
# 5. make defconfig - Konfiguration validieren
# ----------------------------------------
echo "[5/7] Validiere Konfiguration (make defconfig)..."
make defconfig
echo "[5/7] defconfig OK"

# ----------------------------------------
# 6. Toolchain bauen
# ----------------------------------------
echo ""
echo "============================================"
echo "[6/7] TOOLCHAIN BUILD"
echo "      Das dauert 30-90 Minuten!"
echo "      CPU-Kerne: $(nproc)"
echo "============================================"
echo ""
read -p "Toolchain jetzt bauen? (j/n): " answer
if [ "$answer" = "j" ] || [ "$answer" = "J" ] || [ "$answer" = "y" ]; then
    make -j$(nproc) toolchain/compile V=s 2>&1 | tee /tmp/toolchain_build.log
    echo "[6/7] Toolchain gebaut. OK"
else
    echo "[6/7] Toolchain-Build uebersprungen. Spaeter mit:"
    echo "  cd $OPENWRT_DIR && make -j\$(nproc) toolchain/compile V=s"
fi

# ----------------------------------------
# 7. mt76 Source vorbereiten
# ----------------------------------------
echo "[7/7] Bereite mt76-Paket vor..."
make package/kernel/mt76/prepare V=s 2>&1 | tail -5

# Zeige mt76 Source-Verzeichnis
MT76_SRC=$(find build_dir -path "*/mt76-*" -name "mt76.h" 2>/dev/null | head -1 | xargs dirname 2>/dev/null)
if [ -n "$MT76_SRC" ]; then
    echo ""
    echo "============================================"
    echo "MT76 Source extrahiert in:"
    echo "  $MT76_SRC"
    echo ""
    echo "MT7615 Dateien:"
    ls -la "$MT76_SRC/mt7615/" 2>/dev/null | head -20
    echo "============================================"
else
    echo "HINWEIS: mt76 Source wird beim ersten Compile extrahiert."
fi

# ----------------------------------------
# Zusammenfassung
# ----------------------------------------
echo ""
echo "============================================"
echo "Phase 2 ABGESCHLOSSEN"
echo "============================================"
echo "OpenWrt Dir:    $OPENWRT_DIR"
echo "Target:         ramips / mt7621"
echo "Device:         Netgear R6800"
echo "Kernel Version: $(grep 'KERNEL_PATCHVER' $OPENWRT_DIR/target/linux/ramips/Makefile 2>/dev/null || echo 'siehe Makefile')"
echo ""
echo "Naechste Schritte:"
echo "  1. Falls Toolchain noch nicht gebaut:"
echo "     cd $OPENWRT_DIR && make -j\$(nproc) toolchain/compile V=s"
echo "  2. mt76 Modul testen (unveraendert bauen):"
echo "     make package/kernel/mt76/compile V=s"
echo "  3. .ko Datei finden:"
echo "     find build_dir -name 'mt7615e.ko' -o -name 'mt76.ko'"
echo "  4. Dann Phase 3: CSI-Patches erstellen"
echo "============================================"
