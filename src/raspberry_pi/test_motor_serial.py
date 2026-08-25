#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2025
Raspberry Pi 5 to ESP32 Interactive Serial Motor Test Utility

Use this script to test all movement commands with the robot ELEVATED:
1. FORWARD  (DC Motor Forward, Steering Centered 100 deg)
2. BACKWARD (DC Motor Backward, Steering Centered 100 deg)
3. LEFT     (DC Motor Turn Speed, Steering Left 80 deg)
4. RIGHT    (DC Motor Turn Speed, Steering Right 120 deg)
5. STOP     (DC Motor Stop, Steering Centered 100 deg)
6. TIMEOUT FAILSAFE TEST (Send FORWARD once and observe 500ms watchdog shutdown)
"""

import time
import sys
from wro_serial import WROSerialController


def print_banner():
    print("=" * 65)
    print("   ROBOVANGUARD - WRO Future Engineers Serial Motor Test Utility")
    print("   Raspberry Pi 5 -> USB (115200 baud) -> ESP32 Motor Controller")
    print("=" * 65)
    print("[SAFETY WARNING]: Ensure the robot is elevated so wheels spin freely!")
    print("-" * 65)


def print_menu():
    print("\nSelect an action:")
    print("  [1] Send FORWARD  (Continuous stream for 2.0s)")
    print("  [2] Send BACKWARD (Continuous stream for 2.0s)")
    print("  [3] Send LEFT     (Continuous stream for 2.0s)")
    print("  [4] Send RIGHT    (Continuous stream for 2.0s)")
    print("  [5] Send STOP     (Immediate brake)")
    print("  [6] Test Single Command (Immediate 1-shot burst)")
    print("  [7] Test ESP32 Failsafe Timeout (Send 1 FORWARD and wait 1s)")
    print("  [q] Quit test utility")
    print("-" * 65)


def send_timed_stream(controller: WROSerialController, command: str, duration_sec: float = 2.0, interval: float = 0.05):
    """
    Sends continuous commands at 20Hz interval so the ESP32 failsafe watchdog (500ms)
    remains satisfied during active movement.
    """
    print(f"\n[STREAM] Streaming '{command}' for {duration_sec}s...")
    start_time = time.time()
    count = 0
    while time.time() - start_time < duration_sec:
        controller.send_command(command)
        count += 1
        time.sleep(interval)

    # Immediately stop after duration
    print(f"[STREAM] Sent {count} packets. Sending 'STOP'...")
    controller.send_command("STOP")
    time.sleep(0.1)


def main():
    print_banner()

    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    controller = WROSerialController(port=port)
    if not controller.connect():
        print("\n[ERROR] Unable to connect to ESP32. Please check USB cable connection.")
        return

    time.sleep(0.5)

    try:
        while True:
            print_menu()
            choice = input("Enter choice [1-7, q]: ").strip().lower()

            if choice == "1":
                send_timed_stream(controller, "FORWARD", 2.0)
            elif choice == "2":
                send_timed_stream(controller, "BACKWARD", 2.0)
            elif choice == "3":
                send_timed_stream(controller, "LEFT", 2.0)
            elif choice == "4":
                send_timed_stream(controller, "RIGHT", 2.0)
            elif choice == "5":
                print("\n[MANUAL] Sending immediate STOP...")
                controller.send_command("STOP")
            elif choice == "6":
                sub_cmd = input("Enter command to send (FORWARD/BACKWARD/LEFT/RIGHT/STOP): ").strip().upper()
                print(f"[MANUAL] Sending single 1-shot command: {sub_cmd}")
                controller.send_command(sub_cmd)
            elif choice == "7":
                print("\n[FAILSAFE TEST] Sending single 'FORWARD' command without streaming...")
                print("[FAILSAFE TEST] Expected: Robot runs forward for 500ms, then watchdog stops motors automatically.")
                controller.send_command("FORWARD")
                time.sleep(1.2)
                print("[FAILSAFE TEST] Watchdog test complete. Did motors halt after ~500ms?")
            elif choice == "q":
                print("\nExiting test utility...")
                break
            else:
                print("Invalid choice, please try again.")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Aborting...")
    finally:
        print("[SHUTDOWN] Sending safety STOP and closing port...")
        controller.send_command("STOP")
        time.sleep(0.1)
        controller.disconnect()
        print("Goodbye!")


if __name__ == "__main__":
    main()
