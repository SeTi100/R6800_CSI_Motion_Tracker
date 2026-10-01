# ============================================================
# Phase 2 - Schritt 1: WSL2 Installation
# MUSS ALS ADMINISTRATOR ausgefuehrt werden!
# Rechtsklick PowerShell -> "Als Administrator ausfuehren"
# ============================================================

Write-Host "=== R6800 CSI Projekt - WSL2 Setup ===" -ForegroundColor Cyan

# 1. WSL2 + Ubuntu installieren
Write-Host "`n[1/3] Installiere WSL2 mit Ubuntu 24.04..." -ForegroundColor Yellow
wsl --install -d Ubuntu-24.04

Write-Host "`n[2/3] WSL2 Installation gestartet." -ForegroundColor Green
Write-Host "WICHTIG: Nach der Installation:" -ForegroundColor Red
Write-Host "  1. PC NEUSTARTEN (falls gefordert)" -ForegroundColor White
Write-Host "  2. Ubuntu wird beim ersten Start Username/Passwort abfragen" -ForegroundColor White
Write-Host "  3. Dann phase2_openwrt_setup.sh in WSL ausfuehren" -ForegroundColor White

Write-Host "`n[3/3] Setup-Skript fuer nach dem Neustart wird erstellt..." -ForegroundColor Yellow
