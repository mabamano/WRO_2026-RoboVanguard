# Control Software (`src/`)

This directory contains the autonomous control software for the **ROBOVANGUARD** vehicle participating in the WRO Future Engineers competition. The software runs on a **Raspberry Pi 5** using Python 3, OpenCV, `picamera2`, and PySerial to perform real-time vision processing and send steering/speed commands to the ESP32 RoboGuard motor controller.

---

## 📁 File Structure

* [`round1.py`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/src/round1.py) → Implementation for **Round 1 (Open Track Challenge)**. Focuses on high-speed black wall contour tracking, counter-steering, direction indicator line detection (orange/blue), and 3-lap counting.
* [`round2.py`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/src/round2.py) → Implementation for **Round 2 (Obstacle Avoidance & Parking Challenge)**. Features red/green obstacle box detection (red = turn right, green = turn left), line color tracking, lap counting, and automatic parking routine into the parking lot.
* [`my_old_contour_colorvals_crt.py`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/src/my_old_contour_colorvals_crt.py) → Baseline vision processing script with Lab/HSV color thresholding, ROI extraction helper functions, and serial command output dispatcher.

---

## 🛠️ Software Stack & Prerequisites

* **Operating System:** Raspberry Pi OS 64-bit (Debian Bookworm)
* **Python Version:** Python 3.9+
* **Dependencies:**
  * `opencv-python` (OpenCV for image processing and contour extraction)
  * `picamera2` (Official Raspberry Pi Camera Module 3 / Pi Cam library)
  * `pyserial` (Serial communication interface with ESP32)
  * `numpy` (Matrix computations and mask operations)

### Installing Dependencies

```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-pip python3-numpy python3-picamera2
pip3 install pyserial
```

---

## 📡 Serial Communication Protocol

The Raspberry Pi 5 connects to the ESP32 RoboGuard controller via UART on `/dev/serial0` at **115200 baud**. 

Commands are single-character ASCII codes sent periodically or upon state changes:

| Command | ASCII Char | Description |
| :--- | :--- | :--- |
| **Forward** | `'f'` | Drives BO DC motor forward with centered steering |
| **Steer Left** | `'l'` | Pivots MG995 servo left ($\approx -20^\circ$) |
| **Steer Right** | `'r'` | Pivots MG995 servo right ($\approx +20^\circ$) |
| **Stop** | `'s'` | Stops DC motor and centers steering servo |
| **Park** | `'p'` | Initiates parking routine on ESP32 |

---

## 🚀 Execution Instructions

### Running Round 1 (Open Track Challenge)

```bash
cd ~/WRO_2026-RoboVanguard/src
python3 round1.py
```

### Running Round 2 (Obstacle Avoidance & Parking Challenge)

```bash
cd ~/WRO_2026-RoboVanguard/src
python3 round2.py
```

---

## 👥 Lead Developers

* **CV Lead:** Manojkumar M (CSBS 4th Yr, Ramco Institute of Technology)
* **Software Lead:** Abishek Kumar V (CSBS 3rd Yr, Ramco Institute of Technology)