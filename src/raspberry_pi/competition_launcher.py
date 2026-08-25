#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 Competition Launcher Script

Runs on Pi 5 boot:
- Listens on GPIO 17 for physical push button press.
- Ignores stdin EOF when running as background systemd service.
- Launches Round 1 Open Challenge code (open_challenge_R1.py) automatically when button is pressed!
"""

import sys
import os
import time
import subprocess
import select

def wait_for_button(gpio_pin=17, active_high=False):
    print("=" * 65)
    print("   ROBOVANGUARD WRO 2026 COMPETITION LAUNCHER")
    print(f"   Waiting for physical push button press on GPIO {gpio_pin}...")
    print("=" * 65)

    is_interactive = sys.stdin.isatty()
    if is_interactive:
        print("  -> Running in interactive terminal (Press ENTER or Button to start)")
    else:
        print("  -> Running as background systemd service (Waiting strictly for GPIO Button)")

    button_obj = None
    try:
        from gpiozero import Button
        # pull_up=True -> Button wired to GND (active LOW, default for WRO buttons)
        # pull_up=False -> Button wired to 3.3V (active HIGH)
        pull_up = not active_high
        button_obj = Button(gpio_pin, pull_up=pull_up, bounce_time=0.05)
        print(f"[GPIO] gpiozero Button initialized on GPIO {gpio_pin} (pull_up={pull_up}).")
    except Exception:
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            pud = GPIO.PUD_DOWN if active_high else GPIO.PUD_UP
            GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=pud)
            print(f"[GPIO] RPi.GPIO initialized on GPIO {gpio_pin}.")
        except Exception as e:
            print(f"[GPIO WARNING] GPIO library failed ({e}).")

    # Initial settling delay to clear power-on transients
    time.sleep(0.5)

    # Wait for button to be released first if held down during boot
    print("[GPIO] Verifying button state... (Release button if held down)")
    settle_start = time.time()
    while True:
        is_pressed = False
        if button_obj is not None:
            is_pressed = button_obj.is_pressed
        elif 'GPIO' in sys.modules:
            import RPi.GPIO as GPIO
            pin_val = GPIO.input(gpio_pin)
            is_pressed = (pin_val == GPIO.HIGH) if active_high else (pin_val == GPIO.LOW)

        if not is_pressed or (time.time() - settle_start > 3.0):
            break
        time.sleep(0.05)

    print("[GPIO] STANDBY READY! Press physical push button now...")

    # Main wait loop
    while True:
        is_pressed = False
        if button_obj is not None:
            is_pressed = button_obj.is_pressed
        elif 'GPIO' in sys.modules:
            import RPi.GPIO as GPIO
            pin_val = GPIO.input(gpio_pin)
            is_pressed = (pin_val == GPIO.HIGH) if active_high else (pin_val == GPIO.LOW)

        if is_pressed:
            # Confirm press with 80ms debounce hold
            time.sleep(0.08)
            confirm_pressed = False
            if button_obj is not None:
                confirm_pressed = button_obj.is_pressed
            elif 'GPIO' in sys.modules:
                import RPi.GPIO as GPIO
                pin_val = GPIO.input(gpio_pin)
                confirm_pressed = (pin_val == GPIO.HIGH) if active_high else (pin_val == GPIO.LOW)

            if confirm_pressed:
                print("\n[LAUNCH] PHYSICAL BUTTON PRESSED! Launching Round 1 Open Challenge...")
                return True

        # Check terminal stdin ONLY if running interactively in terminal (NOT in systemd background)
        if is_interactive:
            if sys.stdin in select.select([sys.stdin], [], [], 0.02)[0]:
                line = sys.stdin.readline()
                print("\n[LAUNCH] ENTER key pressed in terminal! Launching...")
                return True

        time.sleep(0.05)

def main():
    gpio_pin = 17
    active_high = "--active-high" in sys.argv

    if "--pin" in sys.argv:
        idx = sys.argv.index("--pin")
        if idx + 1 < len(sys.argv):
            gpio_pin = int(sys.argv[idx + 1])

    wait_for_button(gpio_pin, active_high)

    # Path to open_challenge_R1.py script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    r1_script = os.path.join(script_dir, "open_challenge_R1.py")

    # Forward any arguments like --no-display or --webcam
    extra_args = [arg for arg in sys.argv[1:] if arg not in ("--pin", str(gpio_pin), "--active-high")]
    cmd = [sys.executable, r1_script, "--no-wait"] + extra_args
    print(f"[EXEC] Running: {' '.join(cmd)}")
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n[LAUNCHER] Stopped by user.")

if __name__ == "__main__":
    main()
