#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2025
Raspberry Pi 5 Master Autonomous Navigation Node (Round 1)

Architecture:
- High-level decision making, state machines, vision/sensor processing on RPi 5
- Low-level motor & servo actuation commands transmitted to ESP32 via USB Serial
- Control loop runs at 20 Hz (50ms interval), continuously refreshing the 500ms ESP32 watchdog

Commands Sent:
- FORWARD
- BACKWARD
- LEFT
- RIGHT
- STOP
"""

import time
import sys
import signal
from wro_serial import WROSerialController


class WRONavigationMaster:
    """Master Autonomous Controller running on Raspberry Pi 5."""

    def __init__(self, port=None):
        self.serial_ctrl = WROSerialController(port=port)
        self.is_running = False

        # WRO Navigation Parameters (matches original WRO Round 1 parameters)
        self.line_chk_count = 12      # 3 Laps x 4 corners = 12 line detections
        self.line_count = 0
        self.current_command = "STOP"
        self.loop_rate_hz = 20        # 20 Hz control loop (50ms cycle)

        # Setup graceful signal handlers
        signal.signal(signal.SIGINT, self._handle_exit)
        signal.signal(signal.SIGTERM, self._handle_exit)

    def _handle_exit(self, signum, frame):
        print("\n[SAFETY] Interrupted! Sending emergency STOP to ESP32...")
        self.stop_robot()
        self.serial_ctrl.disconnect()
        sys.exit(0)

    def connect(self) -> bool:
        """Initializes connection to ESP32 over USB Serial."""
        return self.serial_ctrl.connect()

    def set_motion(self, command: str):
        """Sets the active navigation command dispatched on each control loop cycle."""
        cmd_clean = command.strip().upper()
        if cmd_clean in WROSerialController.VALID_COMMANDS:
            self.current_command = cmd_clean
            self.serial_ctrl.send_command(self.current_command)

    def stop_robot(self):
        """Emergency stop helper."""
        self.current_command = "STOP"
        self.serial_ctrl.send_command("STOP")

    def execute_turn_sequence(self, turn_direction: str, duration_sec: float = 2.0):
        """
        Executes a timed arc turn when a corner line (Blue/Orange) is detected.
        Continuously streams the command at 20Hz so the ESP32 failsafe stays refreshed.
        """
        print(f"[NAV EVENT] Executing {turn_direction} Turn for {duration_sec}s...")
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.set_motion(turn_direction)
            time.sleep(1.0 / self.loop_rate_hz)

        # Return to forward driving after turn
        self.set_motion("FORWARD")

    def run_autonomous_loop(self):
        """
        Main autonomous control loop.
        Dispatches movement commands to the ESP32 at 20Hz.
        """
        print("\n" + "=" * 60)
        print("   ROBOVANGUARD - Starting WRO Round 1 Autonomous Run")
        print("   Controller: Raspberry Pi 5  |  Actuator: ESP32 over USB")
        print("=" * 60)

        self.is_running = True
        self.line_count = 0

        # Start driving forward
        self.set_motion("FORWARD")

        try:
            while self.is_running and (self.line_count < self.line_chk_count):
                loop_start = time.time()

                # -------------------------------------------------------------
                # High-Level Decision Making (Sensors / Vision / Strategy)
                # -------------------------------------------------------------
                # In full autonomous mode, sensor/camera data processed on RPi 5
                # dictates whether to stay FORWARD, steer LEFT, steer RIGHT, or STOP.
                #
                # Example:
                # if obstacle_detected():
                #     self.set_motion("STOP")
                # elif left_wall_too_close():
                #     self.set_motion("RIGHT")
                # elif right_wall_too_close():
                #     self.set_motion("LEFT")
                # else:
                #     self.set_motion("FORWARD")
                # -------------------------------------------------------------

                # Send active command (refreshes the 500ms ESP32 watchdog)
                self.serial_ctrl.send_command(self.current_command)

                # Maintain 20 Hz loop timing
                elapsed = time.time() - loop_start
                sleep_time = max(0.0, (1.0 / self.loop_rate_hz) - elapsed)
                time.sleep(sleep_time)

            print(f"\n[FINISH] Lap count complete ({self.line_count}/{self.line_chk_count}). Parking...")
            self.stop_robot()

        except KeyboardInterrupt:
            self._handle_exit(None, None)
        finally:
            self.stop_robot()
            self.serial_ctrl.disconnect()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    master = WRONavigationMaster(port=port)
    if master.connect():
        master.run_autonomous_loop()
    else:
        print("[ERROR] Failed to start WRO Master Controller. ESP32 not detected on USB.")


if __name__ == "__main__":
    main()
