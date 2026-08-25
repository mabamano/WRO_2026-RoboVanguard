#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 to ESP32 USB Serial Communication Module

Features:
- USB Serial at 115200 baud
- Automatic port detection (/dev/ttyUSB*, /dev/ttyACM*, COM*)
- Strict command validation (prevents ERROR:UNKNOWN_COMMAND on ESP32)
- Bootloader noise clearing on port open
- Non-blocking ACK reading & telemetry logging
- Auto-reconnection and disconnection fault-tolerance
"""

import time
import threading
import sys
import glob
from typing import Optional, Callable

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 'pyserial' is not installed. Please install it using: pip install pyserial", file=sys.stderr)


class WROSerialController:
    """Manages physical USB serial communication between Raspberry Pi 5 and ESP32."""

    VALID_COMMANDS = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP",
        "AUTO_US_ON", "AUTO_US_OFF", "TURN_LEFT", "TURN_RIGHT"
    }

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 0.1, auto_connect: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.is_running = False
        self._lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self.last_ack: Optional[str] = None
        self.on_ack_callback: Optional[Callable[[str], None]] = None
        self.us_data = {"f": 0, "l": 0, "r": 0, "b": 0}

        if auto_connect:
            self.connect()

    def find_serial_port(self) -> Optional[str]:
        """Automatically detects standard ESP32 USB Serial port across Linux and Windows."""
        if self.port and self.port != "AUTO":
            return self.port

        # Search using serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = p.description.lower()
            if any(k in desc for k in ["cp210", "ch340", "ftdi", "usb serial", "uart", "esp32", "acm"]):
                print(f"[INFO] Auto-detected ESP32 Serial Port: {p.device} ({p.description})")
                return p.device

        # Fallback search for Linux device nodes
        linux_candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if linux_candidates:
            print(f"[INFO] Auto-detected Linux USB Serial Port: {linux_candidates[0]}")
            return linux_candidates[0]

        if ports:
            print(f"[INFO] Defaulting to first available port: {ports[0].device}")
            return ports[0].device

        return None

    def connect(self) -> bool:
        """Establishes USB Serial connection to ESP32."""
        with self._lock:
            if self.serial_conn and self.serial_conn.is_open:
                return True

            target_port = self.find_serial_port()
            if not target_port:
                print("[WARNING] No USB serial port found for ESP32. Retrying on demand.", file=sys.stderr)
                return False

            try:
                print(f"[INFO] Connecting to ESP32 on {target_port} at {self.baudrate} baud...")
                self.serial_conn = serial.Serial(
                    port=target_port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=0.2
                )
                self.port = target_port

                # Allow ESP32 to finish resetting and outputting bootloader text (1.5s)
                time.sleep(1.5)
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                self.is_running = True
                self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
                self._rx_thread.start()

                print(f"[SUCCESS] Connected to ESP32 on {target_port}!")
                return True
            except Exception as e:
                print(f"[ERROR] Failed to connect to {target_port}: {e}", file=sys.stderr)
                self.serial_conn = None
                return False

    def disconnect(self):
        """Safely closes the serial connection."""
        self.is_running = False
        with self._lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(b"STOP\n")
                    self.serial_conn.flush()
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None
        print("[INFO] USB Serial disconnected.")

    def _parse_us_telemetry(self, line: str):
        """Parses ESP32 US telemetry line (e.g. US:F:45,L:28,R:31,B:80)."""
        try:
            parts = line[3:].split(",")
            for p in parts:
                k, v = p.split(":")
                k_clean = k.strip().lower()
                self.us_data[k_clean] = int(v.strip())
        except Exception:
            pass

    def get_us_data(self) -> dict:
        """Returns the latest parsed ultrasonic telemetry dictionary."""
        return self.us_data

    def _read_loop(self):
        """Background thread for asynchronous reading of ACKs and telemetry."""
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode("utf-8", errors="replace").strip()
                        if line:
                            self.last_ack = line
                            if line.startswith("US:"):
                                self._parse_us_telemetry(line)
                            if self.on_ack_callback:
                                self.on_ack_callback(line)
                            elif not line.startswith("US:"):
                                print(f"[ESP32 >> RPi5]: {line}")
                time.sleep(0.01)
            except Exception as e:
                print(f"[WARNING] Serial read error: {e}", file=sys.stderr)
                time.sleep(0.5)

    def send_command(self, command: str) -> bool:
        """
        Sends a movement command to the ESP32.
        Command is strictly validated and formatted with a trailing newline.
        Supports: FORWARD, BACKWARD, LEFT, RIGHT, STOP, AUTO_US_ON, AUTO_US_OFF,
                  TURN_LEFT, TURN_RIGHT, STEER:<angle>, DRIVE:<speed>:<angle>, SET_SPEED:<pwm>
        """
        cmd_clean = command.strip().upper()

        # Strict command validation logic
        parts = cmd_clean.split(":")
        is_valid = False

        if cmd_clean in self.VALID_COMMANDS:
            is_valid = True
        elif cmd_clean.startswith("STEER:") and len(parts) == 2 and parts[1].lstrip('-').isdigit():
            is_valid = True
        elif cmd_clean.startswith("DRIVE:") and len(parts) == 3 and parts[1].lstrip('-').isdigit() and parts[2].lstrip('-').isdigit():
            is_valid = True
        elif cmd_clean.startswith("SET_SPEED:") and len(parts) == 2 and parts[1].lstrip('-').isdigit():
            is_valid = True
        elif cmd_clean.startswith("SET_TURN_DELAY:") and len(parts) == 2 and parts[1].isdigit():
            is_valid = True

        if not is_valid:
            print(f"[ERROR] Rejected invalid command string '{command}'.", file=sys.stderr)
            return False

        message = f"{cmd_clean}\n".encode("utf-8")

        with self._lock:
            if not self.serial_conn or not self.serial_conn.is_open:
                if not self.connect():
                    return False

            try:
                self.serial_conn.write(message)
                self.serial_conn.flush()
                return True
            except (serial.SerialException, OSError) as e:
                print(f"[ERROR] Failed to transmit command '{cmd_clean}' over USB: {e}", file=sys.stderr)
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                    self.serial_conn = None
                return False

    def send_steer(self, angle: int) -> bool:
        """Sends continuous steering angle command (e.g. STEER:100)."""
        return self.send_command(f"STEER:{int(angle)}")

    def send_drive(self, speed: int, angle: int) -> bool:
        """Sends drive command with speed and steering angle (e.g. DRIVE:245:100)."""
        return self.send_command(f"DRIVE:{int(speed)}:{int(angle)}")


_default_controller: Optional[WROSerialController] = None


def get_controller(port: Optional[str] = None) -> WROSerialController:
    """Returns or initializes the global serial controller singleton."""
    global _default_controller
    if _default_controller is None:
        _default_controller = WROSerialController(port=port)
    return _default_controller


def send_command(command: str) -> bool:
    """Convenience function for direct command transmission."""
    controller = get_controller()
    return controller.send_command(command)


if __name__ == "__main__":
    print("Testing WRO Serial Module standalone...")
    ctrl = WROSerialController()
    if ctrl.connect():
        print("Sending test command: FORWARD")
        ctrl.send_command("FORWARD")
        time.sleep(1.0)
        print("Sending test command: STOP")
        ctrl.send_command("STOP")
        time.sleep(0.5)
        ctrl.disconnect()
    else:
        print("Could not connect to ESP32.")
