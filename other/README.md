# Supplementary Materials (`other/`)

This directory contains supplementary documentation, technical datasheets, environment setup guides, and system configuration scripts for **ROBOVANGUARD**.

---

## 📄 Contents

1. **`rpi5_autostart_guide.md`** → Guide on configuring `systemd` services to automatically launch [`round1.py`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/src/round1.py) or [`round2.py`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/src/round2.py) on Raspberry Pi 5 boot.
2. **`serial_config.md`** → Instructions for enabling `/dev/serial0` UART interface on Raspberry Pi 5 (`raspi-config` & `config.txt` settings).
3. **`hardware_datasheets/`** → Reference datasheets for ESP32, DRV8833 motor driver, MG995 servo, TCS34725 color sensor, MT3608 boost converter, and TP4056 charger module.

---

## 🔧 Raspberry Pi 5 Auto-Start Setup

To configure the vision script to start automatically when the start button is pressed:

```ini
# /etc/systemd/system/robovanguard.service
[Unit]
Description=ROBOVANGUARD Autonomous Vision Service
After=multi-user.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/WRO_2026-RoboVanguard/src
ExecStart=/usr/bin/python3 /home/pi/WRO_2026-RoboVanguard/src/round1.py
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robovanguard.service
```
