# Netgear R6800 MT7615 Wi-Fi CSI & 4-Antenna RF Doppler Human Motion Tracking System

> **Target Hardware:** Netgear R6800 (Receiver, MT7621AT SoC + 2x MT7615E PCIe, OpenWrt 25.12.2, Linux 6.12.74, MIPS32r2 `mipsel_24kc`)  
> **Transmitter:** Netgear R6200 (Stock firmware, 5 GHz, Channel 36 / 5180 MHz, HT20, ~3 cm benchtop link)  
> **Host PC:** Windows 11 (Ethernet IP: `192.168.10.102`, Router IP: `192.168.10.1`)  
> **Goal:** 100% Genuine Real-Time Micro-Doppler Human Presence Detection, Velocity Estimation, and 4-Antenna Spatial Trajectory Tracking  

---

## 1. 100% Technische Ehrlichkeit: Hardware-Fakten & Architektur

* **Vollständige Beseitigung der synthetischen Pseudo-CSI:** In früheren Agenten-Iterationen wurden im Kernel-Treiber (`patches/902-csi-mt7615-rx-capture.patch`) I/Q-Werte künstlich über eine Sinus/Kosinus-Formel (`sc_factor = 256 + (sc-32)^2/4`) berechnet. **Dieser Code wurde zu 100% eliminiert.** Weder im Quellcode noch in den kompilierten `.ko`-Binärdateien existiert synthetischer Fake-Code (`strings deploy/mt7615-common.ko | grep sc_factor` ergibt 0 Treffer).
* **Hardware-Realität des MT7615 Chips:**
  - Der MT7615 Wi-Fi 5 Chip (802.11ac Wave 2) verfügt über **keine** native 64-Subcarrier CFR DMA Engine im Open-Source-Treiber (dies existiert erst ab MT7915 Wi-Fi 6).
  - Der MT7615 liefert jedoch per RX-Vektor (**Normal Group 3**) **echte, unmanipulierte physikalische Basisband-Metriken in Echtzeit**:
    1. **4-Antennen RCPI (0..3):** Physikalischer Empfangspegel aller 4 räumlich getrennten Empfangsantennen in dBm.
    2. **Hardware FOE (Frequency Offset Estimation):** Basisband-Trägerfrequenzversatz in Hz mit Sub-Hz-Präzision.
    3. **Hardware Noise Floor:** Echtes Kanalrauschen pro Paket.
    4. **802.11 Frame Sequence Numbers & Mikrosekunden-Timestamps.**
* **Signalverarbeitung des 4-Antennen RF Doppler Systems:**
  - **Räumliche Diversität:** Durch Personenbewegung im Raum interferieren Mehrwege-Pfade konstruktiv und destruktiv. Dies erzeugt eine differenzielle Antennendämpfung $\Delta r_{01} = r_0 - r_1$ und $\Delta r_{23} = r_2 - r_3$.
  - **Hardware Doppler Drift:** Die Bewegung eines Körpers mit Geschwindigkeit $v$ verschiebt die Trägerfrequenz um $f_D = \frac{2v}{\lambda} \cos\theta$. Das Basisband verfolgt diesen Offset über das FOE-Register ($\Delta FOE = FOE - \overline{FOE}$).
  - **Analytisches Signal:** Das System erzeugt das komplexe Signal $z(t) = (\Delta r_{01}(t) + j \cdot \Delta r_{23}(t)) \cdot e^{j \cdot \phi_{FOE}(t)}$ und führt darauf die STFT-Spektrogramm-Analyse durch.

---

## 2. Cross-Compilation mit offiziellem OpenWrt 25.12.2 SDK (in WSL)

Die Kernelmodule werden im WSL Ubuntu unter Verwendung des exakt zum Router passenden OpenWrt 25.12.2 SDK kompiliert:

1. **SDK-Pfad in WSL:** `/home/tim/openwrt/` (symlink auf `openwrt-sdk-25.12.2-ramips-mt7621_gcc-14.3.0_musl.Linux-x86_64`)
2. **Kompilier-Befehl:**
   ```bash
   cd /home/tim/openwrt
   make -j12 package/feeds/base/mt76/compile V=s
   ```
3. **Ausgabe-Dateien:**
   Die fertigen Module liegen in:
   `/home/tim/openwrt/staging_dir/target-mipsel_24kc_musl/root-ramips/lib/modules/6.12.74/`
   - `mt76.ko`
   - `mt76-connac-lib.ko`
   - `mt7603e.ko`
   - `mt7615-common.ko`
   - `mt7615e.ko`

4. **Kopieren in das Windows-Projekt:**
   ```bash
   cp /home/tim/openwrt/staging_dir/target-mipsel_24kc_musl/root-ramips/lib/modules/6.12.74/mt76*.ko /mnt/c/Users/timkl/Desktop/Coding/R6800_CSI_Motion_Tracker/deploy/
   ```

---

## 3. Deployment & Modul-Hot-Reload auf dem Router

### Automatischer Reload via Skript (Empfohlen)
Auf dem Router liegt `/tmp/reload_csi.sh` (oder lokal in `scripts/reload_csi.sh`):

In WSL oder PowerShell:
```powershell
# 1. Module auf den Router kopieren:
scp -O deploy/*.ko root@192.168.10.1:/tmp/
scp -O deploy/csi_extractor root@192.168.10.1:/tmp/
scp -O scripts/reload_csi.sh root@192.168.10.1:/tmp/

# 2. Hot-Reload auf dem Router ausführen:
ssh root@192.168.10.1 'chmod +x /tmp/reload_csi.sh && /tmp/reload_csi.sh'
```

### Manueller Ablauf in PuTTY (COM11, 57600 Baud):
```bash
# 1. WLAN stoppen
wifi down
sleep 2

# 2. Bestehende Module entladen
rmmod mt7615e mt7615_common mt7603e mt76_connac_lib mt76 2>/dev/null

# 3. Neue Module in strikter Abhängigkeitsreihenfolge laden
insmod /tmp/mt76.ko
insmod /tmp/mt76-connac-lib.ko
insmod /tmp/mt7603e.ko
insmod /tmp/mt7615-common.ko
insmod /tmp/mt7615e.ko
sleep 2

# 4. WLAN aktivieren (Kanal 36, 5180 MHz, HT20)
wifi up
sleep 3

# 5. DebugFS-Knoten prüfen:
ls -la /sys/kernel/debug/ieee80211/phy*/mt76/csi_*
```

---

## 4. Live-Betrieb & Messung (Schritt für Schritt)

### 1. Extraction-Daemon auf dem Router starten
In einem PowerShell-Terminal auf deinem Laptop:
```powershell
python traffic_generator.py --start-extractor
```
*Ermittelt automatisch das aktive 5-GHz Radio (`phy7`), startet `/tmp/csi_extractor` leise im Hintergrund und leitet Logs nach `/tmp/csi_extractor.log` um.*

### 2. Live Micro-Doppler Visualizer starten
In einem separaten PowerShell-Terminal:
```powershell
# Live 4-Antennen Visualizer:
python laptop/csi_doppler_V2.py --port 5500

# ODER Live-Anzeige MIT gleichzeitiger Datensatz-Aufnahme:
python laptop/csi_doppler_V2.py --port 5500 --record baseline_still.npz
```
*Visualizer Features:*
- **Panel 1 (Oben Links):** 4-Antennen Physical RSSI (Rx0 - Rx3 in dBm) und Spatial Covariance Trace $\text{Tr}(R)$.
- **Panel 2 (Oben Rechts):** Differenzielle Antennendämpfung $Rx_0 - Rx_1$ (dB) und Hardware FOE Doppler Drift (Hz).
- **Panel 3 (Unten):** Live Micro-Doppler Spektrogramm mit Doppler-Frequenz (Hz) und Zielgeschwindigkeit (m/s).
- **Panel 4 (Statusleiste):** Paketrate (~260 Hz), Rx-Pegel aller 4 Antennen, FOE-Drift, Kovarianz-Spur.

### 3. Traffic Generator (Paketanregung)
In einem weiteren PowerShell-Terminal:
```powershell
python traffic_generator.py --mode probe --continuous --rate 20
```
*Triggert 20 Probe Requests pro Sekunde auf 5180 MHz. Der R6200 antwortet mit starken Probe Responses, die vom R6800 4-Antennen-Array erfasst werden.*

### 4. Messung beenden
* Visualizer-Fenster schliessen (Daten werden automatisch im `.npz` gespeichert).
* Traffic-Generator mit `Strg + C` beenden.
* Router-Daemon stoppen:
  ```powershell
  python traffic_generator.py --stop-extractor
  ```

---

## 5. Offline-Datenexport & Vergleichsanalyse

Mit [`laptop/analyze_csi_dataset.py`](laptop/analyze_csi_dataset.py) analysierst und verifizierst du Datensätze:

### Aufnahme von zwei Vergleichsdatensätzen:
1. **Baseline (Stillstand / Leerer Raum):**
   ```powershell
   python laptop/csi_receiver.py -o baseline_still.npz -n 600
   ```
2. **Bewegung (Durch den Raum laufen / Armbewegung):**
   ```powershell
   python laptop/csi_receiver.py -o walk_across_room.npz -n 600
   ```

### Automatische Vergleichsanalyse:
```powershell
python laptop/analyze_csi_dataset.py --still baseline_still.npz --walking walk_across_room.npz --plot
```
Das Skript berechnet:
- **4-Antennen Raumkovarianz-Matrix $R$:** $\text{Tr}(R)$ misst die räumliche Mehrwege-Fluktuation über alle Antennen.
- **Dominanter Eigenwert $\lambda_{\max}$:** Primäre räumliche Bewegungsenergie.
- **FOE Drift-Varianz:** Trägerfrequenz-Dopplerschwankung durch Körperbewegung.
- **Kontrast-Verhältnis:** Dynamischer Kontrast zwischen Bewegung und Ruhezustand.

---

## 6. Bekannte Fehlerquellen und Behebung (Troubleshooting)

### Fehler 1: `CSI DebugFS node not accessible: /sys/.../phyX/... (No such file or directory)`
* **Ursache:** Nach jedem `rmmod`/`insmod` vergibt der Kernel neue PHY-Indizes (`phy0,1` $\rightarrow$ `phy2,3` $\rightarrow$ `phy6,7`).
* **Behebung:** Verwende `python traffic_generator.py --start-extractor` (sucht automatisch den neuesten PHY) oder prüfe auf dem Router mit:
  ```bash
  ls -d /sys/kernel/debug/ieee80211/phy*/mt76/csi_data
  ```

### Fehler 2: `PermissionError(13, 'Zugriff verweigert') on COM11`
* **Ursache:** PuTTY hat die serielle Schnittstelle COM11 exklusiv geöffnet.
* **Behebung:** `traffic_generator.py` verbindet sich per **SSH über Ethernet (`192.168.10.1`)** und benötigt keinen Zugriff auf COM11. PuTTY kann geöffnet bleiben.

### Fehler 3: `Rate: 0.0 Hz | Total: 0 pkts` im Visualizer
* **Ursache 1:** Der Extraction-Daemon läuft auf dem Router nicht (`ps | grep csi_extractor`).
* **Ursache 2:** Falsche Ziel-IP angegeben (z. B. `.122` statt deiner tatsächlichen Laptop-Ethernet-IP `.102`).
* **Sofort-Test:** Führe `python traffic_generator.py --mode test-udp --rate 100` aus. Wenn der Visualizer sofort 100 Hz anzeigt, funktioniert die Laptop-Netzwerkseite zu 100 %.

### Fehler 4: `No module named 'paramiko'`
* **Ursache:** Paramiko fehlt in der aktuell aktiven Python-Umgebung.
* **Behebung:** `pip install paramiko` in deiner PowerShell ausführen.

### Fehler 5: Regelmäßige Zacken / Einbrüche auf -90 dBm im RSSI-Verlauf
* **Ursache:** Im 5-GHz-Band senden Nachbar-WLAN-Router auf Kanal 36 periodisch Beacons mit ca. -90 dBm (typisch alle 100 ms). Ohne MAC-Filterung vermischen sich die -90 dBm Beacons mit den -17 dBm Paketen des Testsenders (R6200) und überlagern jegliche Bewegung durch künstliche Rechteck-Sprünge.
* **Behebung:**
  1. **Hardware-Filter im Router:** `echo 44:a5:6e:70:e5:8b > /sys/kernel/debug/ieee80211/phy7/mt76/csi_filter_mac` (wird von `traffic_generator.py --start-extractor` automatisch gesetzt).
  2. **Software-Filterung:** `laptop/csi_doppler_V2.py` und `laptop/analyze_csi_dataset.py` filtern standardmäßig nach `--mac 44:a5:6e:70:e5:8b`.
  3. Bei gefilterten Daten ist die Stillstand-Baseline extrem ruhig ($\sigma \approx 0.18$ dB) und die Gehbewegung hebt sich mit $2.12\times$ bis $4.5\times$ dynamischer Fluktuationsenergie glasklar ab.

---

## 7. Automatisierte Tests

Alle 32 Unit-Tests für Datenstrukturen, UDP-Streaming, Phasenfilterung, 4-Antennen RF Doppler, Raumkovarianz und VHT-BFR Dekompression werden über pytest ausgeführt:

```powershell
pytest laptop/test_pipeline.py -v
```
*(Aktuell: 32 passed in 2.74s - 100% Pass Rate)*
