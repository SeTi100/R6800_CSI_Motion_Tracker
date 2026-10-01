#!/bin/bash
# ============================================================
# R6800 CSI Project - Build & Deploy Helper
# Usage: ./build.sh [target]
#   targets: mt76, deploy, hot-reload, full
# ============================================================

set -e

OPENWRT_DIR="${OPENWRT_DIR:-$HOME/openwrt}"
ROUTER_IP="${ROUTER_IP:-192.168.1.1}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEPLOY_DIR="$PROJECT_DIR/deploy"
PATCHES_DIR="$PROJECT_DIR/patches"

# Farben
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log() { echo -e "${GREEN}[BUILD]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err() { echo -e "${RED}[ERROR]${NC} $1"; }

# Patches in OpenWrt-Tree kopieren
sync_patches() {
    log "Synchronisiere Patches..."
    mkdir -p "$OPENWRT_DIR/package/kernel/mt76/patches"
    
    # Nur unsere CSI-Patches (900er Nummern) kopieren
    for patch in "$PATCHES_DIR"/9*.patch; do
        [ -f "$patch" ] || continue
        cp -v "$patch" "$OPENWRT_DIR/package/kernel/mt76/patches/"
    done
    
    log "Patches synchronisiert."
}

# mt76 Kernel-Modul bauen
build_mt76() {
    log "Baue mt76 Kernel-Modul..."
    sync_patches
    
    cd "$OPENWRT_DIR"
    make package/kernel/mt76/clean V=s 2>&1 | tail -3
    make package/kernel/mt76/compile V=s -j$(nproc) 2>&1 | tee /tmp/mt76_build.log | tail -20
    
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        log "Build erfolgreich!"
        extract_modules
    else
        err "Build fehlgeschlagen! Siehe /tmp/mt76_build.log"
        exit 1
    fi
}

# .ko Dateien extrahieren
extract_modules() {
    log "Extrahiere Kernel-Module..."
    mkdir -p "$DEPLOY_DIR"
    
    find "$OPENWRT_DIR/build_dir" -name "mt76.ko" 2>/dev/null | head -1 | while read f; do
        cp -v "$f" "$DEPLOY_DIR/"
    done
    
    find "$OPENWRT_DIR/build_dir" -name "mt7615e.ko" 2>/dev/null | head -1 | while read f; do
        cp -v "$f" "$DEPLOY_DIR/"
    done
    
    find "$OPENWRT_DIR/build_dir" -name "mt7615-common.ko" 2>/dev/null | head -1 | while read f; do
        cp -v "$f" "$DEPLOY_DIR/"
    done

    find "$OPENWRT_DIR/build_dir" -name "mt76-connac*.ko" 2>/dev/null | head -1 | while read f; do
        cp -v "$f" "$DEPLOY_DIR/"
    done

    find "$OPENWRT_DIR/build_dir" -name "mt7603e.ko" 2>/dev/null | head -1 | while read f; do
        cp -v "$f" "$DEPLOY_DIR/"
    done

    ls -la "$DEPLOY_DIR/"*.ko 2>/dev/null && log "Module bereit." || warn "Keine .ko Dateien gefunden"
}

# Module zum Router kopieren
deploy() {
    log "Deploye Module zu $ROUTER_IP..."
    scp -O "$DEPLOY_DIR"/*.ko root@$ROUTER_IP:/tmp/
    log "Module auf Router kopiert."
}

# Hot-Reload: Module auf dem Router austauschen
hot_reload() {
    deploy
    log "Hot-Reload auf Router..."
    ssh root@$ROUTER_IP << 'REMOTE'
        echo "Stoppe WiFi..."
        wifi down 2>/dev/null
        sleep 1
        
        echo "Entlade alte Module..."
        rmmod mt7615e 2>/dev/null || true
        rmmod mt7615_common 2>/dev/null || true
        rmmod mt7603e 2>/dev/null || true
        rmmod mt76_connac_lib 2>/dev/null || true
        rmmod mt76 2>/dev/null || true
        sleep 1
        
        echo "Lade neue Module..."
        insmod /tmp/mt76.ko
        [ -f /tmp/mt76_connac_lib.ko ] && insmod /tmp/mt76_connac_lib.ko || true
        [ -f /tmp/mt76-connac-lib.ko ] && insmod /tmp/mt76-connac-lib.ko || true
        [ -f /tmp/mt76-connac.ko ] && insmod /tmp/mt76-connac.ko || true
        [ -f /tmp/mt7603e.ko ] && insmod /tmp/mt7603e.ko || true
        [ -f /tmp/mt7615-common.ko ] && insmod /tmp/mt7615-common.ko || true
        insmod /tmp/mt7615e.ko
        
        echo "Starte WiFi..."
        wifi up
        
        echo "Status:"
        lsmod | grep mt76
        dmesg | tail -10
REMOTE
    log "Hot-Reload abgeschlossen."
}

# Volles Firmware-Image bauen
build_full() {
    log "Baue volles Firmware-Image..."
    sync_patches
    cd "$OPENWRT_DIR"
    make -j$(nproc) V=s 2>&1 | tee /tmp/full_build.log | tail -30
    
    log "Firmware-Image:"
    find bin/targets -name "*r6800*sysupgrade*" -type f 2>/dev/null
}

# ============================================================
# Main
# ============================================================
case "${1:-mt76}" in
    mt76)       build_mt76 ;;
    deploy)     deploy ;;
    hot-reload) build_mt76 && hot_reload ;;
    reload)     hot_reload ;;
    full)       build_full ;;
    extract)    extract_modules ;;
    sync)       sync_patches ;;
    *)
        echo "Usage: $0 {mt76|deploy|hot-reload|reload|full|extract|sync}"
        echo ""
        echo "  mt76       - Build mt76 kernel module only"
        echo "  deploy     - Copy .ko files to router via SCP"
        echo "  hot-reload - Build + deploy + reload on router"
        echo "  reload     - Deploy + reload (skip build)"
        echo "  full       - Build full firmware image"
        echo "  extract    - Extract .ko from last build"
        echo "  sync       - Sync patches to OpenWrt tree"
        exit 1
        ;;
esac
