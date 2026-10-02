# Netgear R6800 MT7615 Wi-Fi CSI Human Motion Tracking System

> **Target Hardware:** Netgear R6800 (Receiver, MT7621AT SoC + 2x MT7615E PCIe, OpenWrt 25.12.2, Linux 6.12.74, MIPS32r2)  
> **Transmitter:** Netgear R6200 (Stock firmware, 5 GHz, Channel 36 / 5180 MHz, HT20, ~3 cm benchtop link)  
> **Host PC:** Windows 11 (Ethernet IP: `192.168.10.102`, Router IP: `192.168.10.1`)  
> **Goal:** Real-time Micro-Doppler Human Presence Detection, Velocity Estimation, and Trajectory Tracking  

---

## 1. 100% Technische Ehrlichkeit: Was ist echt vs. was war Fake

* **Beseitigung der synthetischen Pseudo-CSI:** In früheren Agenten-Iterationen wurden im Kernel-Treiber ([`patches/902-csi-mt7615-rx-capture.patch`](patches/902-csi-mt7615-rx-capture.patch)) I/Q-Werte künstlich über eine Sinus/Kosinus-Formel aus skalarer `RSSI` und `FOE` berechnet. **Dieser Code wurde vollständig entfernt.**
* **Reale Datenbasis:** Der MT7615 Hardware-RX-Statusvektor (Normal Group 3) liefert genuine skalare Metriken (Antennen-RSSI0..3, FOE, Noise Floor). Subcarrier-I/Q-Puffer werden für Standard-RX-Frames genullt.
* **Echte physikalische CFR-Matrizen:** Echte komplexe Kanalübertragungsfunktionen $H(f)$ werden über den IEEE 802.11ac **VHT Compressed Beamforming Report (BFR)** Dekompressor ([`laptop/vht_bfr_decompressor.py`](laptop/vht_bfr_decompressor.py)) via Givens-Rotationsmatrizen mit vollständiger Phasenerhaltung rekonstruiert.

---

## 2. Exakter Standard-Workflow (Schritt für Schritt)

### Schritt 1: Modul-Deployment auf den Router (nach Änderungen)
In PowerShell auf deinem PC:
```powershell
scp -O C:\Users\timkl\Desktop\Coding\R6800_CSI_Motion_Tracker\deploy\*.ko root@192.168.10.1:/tmp/
scp -O C:\Users\timkl\Desktop\Coding\R6800_CSI_Motion_Tracker\deploy\csi_extractor root@192.168.10.1:/tmp/
```

### Schritt 2: Modul-Aktivierung auf dem Router (via PuTTY auf COM11)
Kopiere diesen Block in deine PuTTY-Konsole:
```bash
# 1. WLAN stoppen
wifi down

# 2. Alte Module entladen
rmmod mt7615e 2>/dev/null
rmmod mt7615_common 2>/dev/null
rmmod mt7603e 2>/dev/null
rmmod mt76_connac_lib 2>/dev/null
rmmod mt76 2>/dev/null

# 3. Neue Module in exakter Abhängigkeitsreihenfolge laden
insmod /tmp/mt76.ko
insmod /tmp/mt76-connac-lib.ko
insmod /tmp/mt7603e.ko
insmod /tmp/mt7615-common.ko
insmod /tmp/mt7615e.ko

# 4. WLAN wieder starten (fixiert auf HT20 Kanal 36)
wifi up

# 5. Extraction Daemon ausführbar machen
chmod +x /tmp/csi_extractor
```

---

## 3. Live-Betrieb & Messung (Vollautomatisch ohne PuTTY-Spam)

Dank der automatischen SSH-Steuerung über Ethernet musst du in PuTTY **nichts mehr starten**. PuTTY kann auf COM11 geöffnet bleiben.

### 1. Extraction-Daemon auf dem Router starten
In einem PowerShell-Terminal auf deinem PC:
```powershell
python traffic_generator.py --start-extractor
```
*Ermittelt automatisch das aktive 5-GHz Radio (`phy5`), startet `/tmp/csi_extractor` leise im Hintergrund und leitet Logs nach `/tmp/csi_extractor.log` um.*

### 2. Live Micro-Doppler Visualizer starten
In einem separaten PowerShell-Terminal:
```powershell
# Nur Live-Anzeige:
python laptop/csi_doppler_V2.py --port 5500

# ODER Live-Anzeige MIT gleichzeitiger Datensatz-Aufnahme (.npz):
python laptop/csi_doppler_V2.py --port 5500 --record meine_messung.npz
```

### 3. R6200 kontinuierlich anregen (Traffic Generator)
In einem weiteren PowerShell-Terminal:
```powershell
python traffic_generator.py --mode probe --continuous --rate 20
```
*Triggert 20 Probe Requests pro Sekunde auf 5180 MHz. Der R6200 antwortet mit starken Probe Responses (-9 dBm), die vom CSI-Treiber erfasst werden.*

### 4. Messung beenden
* Visualizer-Fenster schliessen (gespeichert wird automatisch beim Beenden).
* Traffic-Generator mit `Strg + C` stoppen.
* Router-Daemon sauber beenden:
  ```powershell
  python traffic_generator.py --stop-extractor
  ```

---

## 4. Wie prüfe ich, ob die Daten sinnvoll & echt sind?

Es gibt **3 unbestechliche physikalische Kriterien**:

| Kriterium | Erwartetes physikalisches Verhalten | Synthetische Täuschung (früher) |
|---|---|---|
| **1. Frequenzselektivität über Subcarrier** | Amplitudenkurve über die 64 Töne ist unregelmässig zerklüftet mit Minima (Fading Notches) und Maxima (Mehrwege-Interferenz). | Perfekt glatte, identische Sinuswellen über alle 64 Töne. |
| **2. Clutter-Rejection (Stiller Raum)** | Wenn niemand im Raum ist, zeigt das Spektrogramm eine saubere flache Nulllinie (0 Hz). Keine Doppler-Peaks. | Permanentes Rauschen / Drift durch Oszillator-Wärme. |
| **3. Vorzeichen des Doppler-Shifts** | **Annäherung an Router:** Positive Doppler-Frequenz ($+15$ bis $+50\text{ Hz}$).<br>**Entfernung vom Router:** Negative Doppler-Frequenz ($-15$ bis $-50\text{ Hz}$). | Reine RSSI-Fluktuation kann das Vorzeichen (Richtung) nicht auflösen. |

---

## 5. Offline-Datenexport & Vergleichsanalyse

Mit dem Tool [`laptop/analyze_csi_dataset.py`](laptop/analyze_csi_dataset.py) kannst du Aufnahmen offline wissenschaftlich analysieren und vergleichen.

### Aufnahme von zwei Vergleichsdatensätzen:

1. **Datensatz 1: Baseline (Stillstand / Leerer Raum)**
   Starte die Aufnahme für 30 Sekunden, während du dich nicht bewegst:
   ```powershell
   python laptop/csi_receiver.py -o baseline_still.npz -n 600
   ```
   *(Oder via `csi_doppler_V2.py --record baseline_still.npz`)*

2. **Datensatz 2: Bewegung (Durch den Raum laufen)**
   Starte die Aufnahme und laufe normal zwischen den Antennen hin und her:
   ```powershell
   python laptop/csi_receiver.py -o walk_across_room.npz -n 600
   ```

### Automatische Vergleichsanalyse:
```powershell
python laptop/analyze_csi_dataset.py --still baseline_still.npz --walking walk_across_room.npz --plot
```
Das Skript berechnet:
* **Dynamische Bewegungsenergie:** Verhältnis der Doppler-Energie im Bewegungsband ($2 - 40\text{ Hz}$) im Vergleich zum Ruhezustand.
* **Frequenzselektivitäts-Index:** Räumliche Mehrwege-Struktur über die Subcarrier.
* **Grafischer Vergleich:** Zeitverlauf von RSSI und Amplitudenprofilen nebeneinander.

---

## 6. Bekannte Fehlerquellen und deren Behebung (Troubleshooting)

### Fehler 1: `CSI DebugFS node not accessible: /sys/.../phy3/... (No such file or directory)`
* **Ursache:** Jedes Mal, wenn die Module per `rmmod` entladen und per `insmod` neu geladen werden, vergibt der Linux-Kernel neue, fortlaufende PHY-Nummern (`phy0,1` $\rightarrow$ `phy2,3` $\rightarrow$ `phy4,5`). Das 5-GHz Radio heisst dann nicht mehr `phy3`, sondern `phy5`!
* **Behebung:** Verwende `python traffic_generator.py --start-extractor` (sucht automatisch den richtigen PHY) oder ermittle ihn in PuTTY dynamisch:
  ```bash
  PHY=$(ls -d /sys/kernel/debug/ieee80211/phy*/mt76/csi_data | tail -1 | cut -d/ -f6)
  /tmp/csi_extractor -i $PHY -d 192.168.10.102 -p 5500 -e > /dev/null 2>&1 &
  ```

### Fehler 2: `PermissionError(13, 'Zugriff verweigert') on COM11`
* **Ursache:** PuTTY hat die serielle Schnittstelle COM11 exklusiv geöffnet. Unter Windows kann kein zweites Programm gleichzeitig auf COM11 zugreifen.
* **Behebung:** Die `traffic_generator.py` verbindet sich standardmässig per **SSH über Ethernet (`192.168.10.1`)** und benötigt COM11 nicht mehr. Du kannst PuTTY einfach geöffnet lassen.

### Fehler 3: PuTTY-Konsole wird mit `[CSI] Rate: ...` Zeilen überflutet
* **Ursache:** `csi_extractor` wurde mit `&` im Hintergrund gestartet, aber `stdout` wurde nicht umgeleitet.
* **Behebung:** Alle Geisterprozesse stoppen mit `killall csi_extractor` und den Daemon mit `> /dev/null 2>&1 &` oder bequem via `python traffic_generator.py --start-extractor` starten.

### Fehler 4: `Rate: 0.0 Hz | Total: 0 pkts` im Visualizer
* **Ursache 1:** Der Extraction-Daemon läuft auf dem Router nicht (`ps | grep csi_extractor`).
* **Ursache 2:** Falsche Ziel-IP angegeben (z. B. `.122` statt deiner tatsächlichen Laptop-Ethernet-IP `.102`).
* **Sofort-Test:** Führe `python traffic_generator.py --mode test-udp --rate 100` aus. Wenn der Visualizer sofort 100 Hz anzeigt, funktioniert die Laptop-Netzwerkseite zu 100 %.

### Fehler 5: `No module named 'paramiko'`
* **Ursache:** Paramiko fehlt in der aktuell aktiven Python-Umgebung.
* **Behebung:** Führe `pip install paramiko` in deiner PowerShell aus.
