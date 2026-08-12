# Electromechanical Diagrams (`schemes/`)

This directory contains circuit schematics, electromechanical block diagrams, and wiring layouts for the **ROBOVANGUARD** vehicle.

---

## ⚡ System Architecture Overview

The vehicle power and control system is divided into high-level processing (**Raspberry Pi 5**), real-time control (**ESP32 RoboGuard Unit**), actuation, and multi-sensor feedback.

```
                  +-------------------------------+
                  |  3.7V 3200 mAh Li-ion Battery |
                  +---------------+---------------+
                                  |
                           [ TP4056 Charger ]
                                  |
                     +------------+------------+
                     |                         |
            [ MT3608 5V Boost ]       [ MT3608 3.3V Step-Down/Up ]
                     |                         |
                     v                         v
            +-----------------+       +-----------------+
            | Raspberry Pi 5  |<=====>| ESP32 RoboGuard |
            |  & Pi Camera    | UART  | Main Controller |
            +-----------------+       +--------+--------+
                                               |
         +-------------------------------------+-------------------------------------+
         |                      |                      |                             |
         v                      v                      v                             v
  [ BO DC Motor ]       [ MG995 Servo ]      [ 6x Ultrasonic Sensors ]    [ TCS34725 Color Sensor ]
   Rear Propulsion       Ackermann Steering    Front (3), Rear (1),         Orange, Blue, Magenta
   (DRV8833 Driver)      (PWM GPIO 18)         Left (1), Right (1)           Color Zone Detection
```

---

## 🔌 Pin Mapping Table

| Component | Interface / Protocol | Connected Pins (ESP32 / Pi 5) | Notes |
| :--- | :--- | :--- | :--- |
| **Raspberry Pi 5** | Serial UART | TX (GPIO 14) / RX (GPIO 15) $\leftrightarrow$ ESP32 RX/TX | High-speed command transmission @ 115200 baud |
| **Pi Camera Module** | CSI Ribbon | Raspberry Pi 5 CSI Port 0 | 640x480 resolution @ 30 FPS for vision |
| **BO DC Motor** | PWM / DRV8833 | GPIO 25, GPIO 26 | Rear wheel propulsion |
| **MG995 Servo** | PWM (50 Hz) | GPIO 18 | Ackermann steering angle control ($\pm 20^\circ$) |
| **Ultrasonic Front (3)** | Trigger / Echo | GPIO 4, 5, 12, 13, 14, 15 | Obstacle & wall distance measurement |
| **Ultrasonic Sides (2)** | Trigger / Echo | GPIO 16, 17, 19, 21 | Left/Right lane centering |
| **Ultrasonic Rear (1)** | Trigger / Echo | GPIO 22, 23 | Reverse safety & parking distance |
| **TCS34725 Color Sensor** | I2C | SDA (GPIO 21), SCL (GPIO 22) | Track line color detection |

---

## 📄 Schematics Included

* `block_diagram.png` → High-level electrical system block diagram.
* `wiring_schematic.pdf` → Detailed pin-to-pin wiring diagram.