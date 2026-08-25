# Control Software (`src/`)

This directory contains the control software for the **ROBOVANGUARD** autonomous vehicle, split into low-level ESP32 motor control firmware and high-level Raspberry Pi 5 computer vision scripts.

---

## 📁 Directory & File Structure

### 1. Raspberry Pi 5 Code (`src/raspberry_pi/`)
High-level control software running on Raspberry Pi OS. Performs webcam image capture, color space masking, contour processing, and sends ASCII serial instructions to the ESP32 controller.

* **`open_challenge_R1.py`** & **`open_challenge_R1_test_1lap.py`**: Main scripts for **Round 1 (Open Track Challenge)**. Uses dual-ROI black wall contour analysis and PD-based wall following.
* **`obstacle_challenge_R2.py`** & **`obstacle_challenge_R2_test_parking.py`**: Main scripts for **Round 2 (Obstacle Avoidance & Parking Challenge)**. Detects Red and Green obstacle boxes, tracks blue/orange orientation lines, and handles automated magenta parking lot alignment.
* **`wro_serial.py`**: Manages USB serial communication with auto-port detection (`/dev/ttyUSB*`, `/dev/ttyACM*`, `COM*`), noise clearing, command validation, and telemetry parsing.
* **`wro_functions.py`**: Common utilities for lane tracking, error calculation, and control parameters.
* **`masks.py`**: HSV color space mask ranges for Red, Green, Orange, Blue, and Magenta tracking.
* **`camera_streamer.py`**: Background Live MJPEG debugger. Hosts a local webserver at `http://<pi_ip>:8080` to display processed frames and overlay metadata.
* **`competition_launcher.py`**: Daemon process listening on GPIO 17 for a physical start button to boot scripts autonomously.
* **`test_webcam.py`** & **`test_motor_serial.py`**: Diagnostic tools to verify webcam feed indices and serial commands.
* **`wro_autostart.service`** & **`install_autostart.sh`**: Systemd integration scripts to automatically start the competition launcher at system boot.

### 2. ESP32 low-level Controller (`src/ROBOVANGUARD_WRO_Round_1_Code_Final/`)
Arduino/C++ firmware compiled and uploaded to the ESP32 RoboGuard controller.

* **`ROBOVANGUARD_WRO_Round_1_Code_Final.ino`**: Manages serial reading, low-level state machines, and DC motor propulsion.
* **`Lib_Declarations_Setup.ino`**: Configures hardware pins, PWM channels for MG995 steering servo, and ultrasonic sensor trigger/echo loops.

---

## 🛠️ Software Stack & Dependencies

The Pi 5 vision system uses a standard **USB Webcam** running OpenCV.

### Prerequisites (Raspberry Pi 5)
Ensure Python 3 and the required libraries are installed:
```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-pip
pip3 install pyserial gpiozero
```

---

## 📡 Serial Commands

The Pi 5 sends command strings to the ESP32 at **115200 baud**:
* `"FORWARD"` - Move forward (steering centered)
* `"BACKWARD"` - Reverse vehicle
* `"LEFT"` / `"RIGHT"` - Steer left/right dynamically
* `"STOP"` - Halt vehicle and center steering
* `"TURN_LEFT"` / `"TURN_RIGHT"` - Hard fixed turns around corners
* `"AUTO_US_ON"` / `"AUTO_US_OFF"` - Enable/disable ultrasonic wall-following override on ESP32

---

## 🚀 How to Run

1. **Verify Webcam and Serial Connections:**
   ```bash
   python3 raspberry_pi/test_webcam.py
   python3 raspberry_pi/test_motor_serial.py
   ```
2. **Execute Challenge Run:**
   - For Round 1: `python3 raspberry_pi/open_challenge_R1.py`
   - For Round 2: `python3 raspberry_pi/obstacle_challenge_R2.py`
3. **Automated Button Launch Setup:**
   Run the installation script to configure system boot behavior:
   ```bash
   sudo chmod +x raspberry_pi/install_autostart.sh
   ./raspberry_pi/install_autostart.sh
   ```