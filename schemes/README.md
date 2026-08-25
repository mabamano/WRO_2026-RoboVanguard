# Electromechanical Diagrams (`schemes/`)

This directory contains circuit schematics, electromechanical block diagrams, and wiring layouts for the **ROBOVANGUARD** vehicle.

---

## 📁 Files in this Directory

* **[`PCB-bottom.jpeg`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/schemes/PCB-bottom.jpeg)**: Bottom routing and pad layout for the low-level controller distribution board.
* **[`chassie-top.jpeg`](file:///c:/Users/mabam/OneDrive/Desktop/WRO2026/WRO-repo/WRO_2026-RoboVanguard/schemes/chassie-top.jpeg)**: Component layout and positioning on the top plate of the vehicle chassis.
* **Download full files:** [Google Drive Schematics Folder](https://drive.google.com/drive/folders/155-CGwuJX2tviMi_41Z1woC64YpkhwIw?usp=sharing)

---

## ⚡ System Power & Architecture Overview

The vehicle uses a split power design to isolate the high-current motor drivers from the high-load vision processors.

* **High-Level Vision:** The **Raspberry Pi 5** and **USB Webcam** are powered by the **DFRobot Pi5 UPS HAT** connected to a dedicated high-drain Li-ion battery source. This setup ensures the Pi 5 never experiences power brownouts during sudden motor start/stop transients.
* **Low-Level Control:** The **ESP32 RoboGuard Unit**, sensors, and motors are powered from a separate 3.7V 3200 mAh Li-ion battery via MT3608 boost regulators.

```
+------------------------------------+          +--------------------------------------+
|       3.7V Li-ion Battery Source   |          |      High-Drain Li-ion Battery       |
+-----------------+------------------+          +------------------+-------------------+
                  |                                                |
          [ TP4056 Charger ]                             [ DFRobot Pi5 UPS HAT ]
                  |                                                |
        [ MT3608 5V/3.3V Boost ]                                   |
                  |                                                |
                  v                                                v
       +--------------------+                            +--------------------+
       |  ESP32 RoboGuard   |<==========================>|   Raspberry Pi 5   |
       |  Main Controller   |         USB Serial         |    & USB Webcam    |
       +---------+----------+                            +--------------------+
                 |
       +---------+-----------------------+-----------------------+
       |                                 |                       |
       v                                 v                       v
[ BO DC Motor ]                   [ MG995 Servo ]      [ Sensors & Indicators ]
Rear propulsion via               Ackermann Steering   Ultrasonic array & TCS34725
DRV8833 driver                    (PWM Control)        Color Sensor
```

---

## 🔌 Pin Mapping Table

| Component | Interface / Protocol | Connected Pins (ESP32 / Pi 5) | Notes |
| :--- | :--- | :--- | :--- |
| **Raspberry Pi 5** | USB Serial | USB Port $\leftrightarrow$ ESP32 micro-USB port | High-speed command transmission @ 115200 baud |
| **USB Webcam** | UVC USB Interface | USB 3.0 Port (Blue) on Pi 5 | 640x480 resolution @ 30 FPS for vision |
| **BO DC Motor** | PWM / DRV8833 | GPIO 25, GPIO 26 | Rear wheel propulsion |
| **MG995 Servo** | PWM (50 Hz) | GPIO 18 | Ackermann steering angle control |
| **Ultrasonic Front (3)** | Trigger / Echo | GPIO 4, 5, 12, 13, 14, 15 | Obstacle & wall distance measurement |
| **Ultrasonic Sides (2)** | Trigger / Echo | GPIO 16, 17, 19, 21 | Left/Right lane centering |
| **Ultrasonic Rear (1)** | Trigger / Echo | GPIO 22, 23 | Reverse safety & parking distance |
| **TCS34725 Color Sensor** | I2C | SDA (GPIO 21), SCL (GPIO 22) | Track line color detection |
