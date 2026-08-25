#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 Open Challenge Autonomous Navigation (Round 1)

WRO 2026 Competition Run Architecture:
1. Corner Steering Calibration: 60° = LEFT TURN, 140° = RIGHT TURN (matching ESP32 firmware).
2. Competition Button Trigger: Pi 5 idles waiting for physical GPIO button press (default GPIO 17).
3. Non-Interactive Safeguard: Bypasses terminal stdin EOF checks when running as background systemd service.
4. ESP32 Handshake: ESP32 stays in idle STOP state until Pi 5 button is pressed.
5. Precision Finish Line Stop: Uses speed 175 + active electronic reverse brake pulse to halt 0cm offset at start line.
"""

import sys
import time
import select
import cv2
import numpy as np
from wro_serial import WROSerialController
from masks import rOrange, rBlack, rBlue
from wro_functions import (CameraManager, find_black_wall_contours, find_contours, max_contour, draw_roi,
                           draw_offset_contours, display_variables)


def wait_for_button_press(gpio_pin=17, show_display=False, window_name="", camera=None, active_high=False):
    """Waits for a physical push button press on Pi 5 GPIO pin before starting autonomous run."""
    print("=" * 65)
    print(f"[COMPETITION STANDBY] Waiting for physical button press on GPIO {gpio_pin}...")
    print("=" * 65)

    is_interactive = sys.stdin.isatty()
    if is_interactive:
        print("  -> Running in interactive terminal (Press ENTER or Button to start)")
    else:
        print("  -> Running as background service (Waiting strictly for physical GPIO Button)")

    button_obj = None
    try:
        from gpiozero import Button
        pull_up = not active_high
        button_obj = Button(gpio_pin, pull_up=pull_up, bounce_time=0.05)
        print(f"[GPIO] Listening on GPIO {gpio_pin} (pull_up={pull_up} via gpiozero).")
    except Exception:
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            pud = GPIO.PUD_DOWN if active_high else GPIO.PUD_UP
            GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=pud)
            print(f"[GPIO] Listening on GPIO {gpio_pin} (via RPi.GPIO).")
        except Exception as e:
            print(f"[GPIO INFO] Hardware GPIO library not loaded ({e}).")

    time.sleep(0.5)

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

    while True:
        is_pressed = False
        if button_obj is not None:
            is_pressed = button_obj.is_pressed
        elif 'GPIO' in sys.modules:
            import RPi.GPIO as GPIO
            pin_val = GPIO.input(gpio_pin)
            is_pressed = (pin_val == GPIO.HIGH) if active_high else (pin_val == GPIO.LOW)

        if is_pressed:
            time.sleep(0.08)
            confirm_pressed = False
            if button_obj is not None:
                confirm_pressed = button_obj.is_pressed
            elif 'GPIO' in sys.modules:
                import RPi.GPIO as GPIO
                pin_val = GPIO.input(gpio_pin)
                confirm_pressed = (pin_val == GPIO.HIGH) if active_high else (pin_val == GPIO.LOW)

            if confirm_pressed:
                print("\n[BUTTON] PHYSICAL BUTTON PRESSED! Launching Competition Run...")
                return True

        if is_interactive:
            if sys.stdin in select.select([sys.stdin], [], [], 0.02)[0]:
                line = sys.stdin.readline()
                print("\n[TERMINAL] Start command received via ENTER key! Launching...")
                return True

        if show_display and window_name and camera:
            img = camera.capture_array()
            if img is not None:
                img_disp = img.copy()
                cv2.putText(img_disp, f"COMPETITION STANDBY (GPIO {gpio_pin})", (30, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(img_disp, "PRESS BUTTON OR 'S' TO START!", (30, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(window_name, img_disp)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord('s'), ord('S'), 13, 32):
                    print("\n[DISPLAY] Start key pressed! Launching Competition Run...")
                    return True
                elif key in (ord('q'), ord('Q'), 27):
                    print("\n[USER CANCEL] Startup aborted.")
                    sys.exit(0)
        else:
            time.sleep(0.05)


def main():
    print("=" * 65)
    print("   ROBOVANGUARD - WRO Round 1 Open Challenge Node (Pi 5)")
    print("   Architecture: Corrected Steering Geometry (60=LEFT, 140=RIGHT)")
    print("=" * 65)

    force_webcam = "--webcam" in sys.argv or "-w" in sys.argv
    use_vision_walls = "--vision-walls" in sys.argv or "--no-us" in sys.argv
    skip_button = "--no-wait" in sys.argv or "--nowait" in sys.argv
    active_high = "--active-high" in sys.argv

    gpio_pin = 17
    if "--pin" in sys.argv:
        idx = sys.argv.index("--pin")
        if idx + 1 < len(sys.argv):
            gpio_pin = int(sys.argv[idx + 1])

    forced_dir = "none"
    if "--dir" in sys.argv:
        idx = sys.argv.index("--dir")
        if idx + 1 < len(sys.argv):
            forced_dir = sys.argv[idx + 1].lower()
            print(f"[CONFIG] Forcing fixed track direction: {forced_dir.upper()}")

    # 1. Initialize USB Serial connection to ESP32
    serial_ctrl = WROSerialController()
    if not serial_ctrl.connect():
        print("[ERROR] Cannot proceed without ESP32 serial connection.")
        sys.exit(1)

    print("[SAFETY] ESP32 connected! Setting robot to idle STOP state...")
    serial_ctrl.send_command("STOP")
    time.sleep(0.5)

    show_monitor_display = "--no-display" not in sys.argv
    window_name = "WRO Open Challenge - Competition Monitor Debug (Pi 5)"

    if show_monitor_display:
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 800, 600)
            print("[DISPLAY] Created OpenCV live display window on monitor!")
        except Exception as e:
            print(f"[WARNING] Could not open GUI display window: {e}")
            show_monitor_display = False

    # 2. Initialize Camera (Picamera2 or USB Webcam)
    camera = CameraManager(force_webcam=force_webcam, device_index=0)
    camera.start()

    print("[INFO] Capturing camera warmup frames...")
    for _ in range(15):
        warmup_frame = camera.capture_array()
        if warmup_frame is not None:
            if show_monitor_display:
                cv2.imshow(window_name, warmup_frame)
                cv2.waitKey(1)
        time.sleep(0.04)

    # ------------------------------------------------------------------------
    # COMPETITION STARTUP: Wait for physical button press on Pi 5 GPIO pin
    # ------------------------------------------------------------------------
    if not skip_button:
        wait_for_button_press(gpio_pin=gpio_pin, show_display=show_monitor_display,
                              window_name=window_name, camera=camera, active_high=active_high)

    # ------------------------------------------------------------------------
    # Phase 1: Setup & Warmup Countdown
    # ------------------------------------------------------------------------
    print("\n[READY] Competition Run Started!")
    print("[PHASE 1] Recording Baseline Start Position Snapshot...")

    start_snapshot = {"f": 0, "f1": 0, "f2": 0, "l": 0, "r": 0, "b": 0}

    print("[COUNTDOWN] Bot starts driving in 3 seconds... (Press 'q' to abort, 'l'/'r' to set dir)")
    for c in range(3, 0, -1):
        print(f"[COUNTDOWN] {c}...")
        us_data = serial_ctrl.get_us_data()
        f_val = us_data.get("f", 0)
        start_snapshot = {
            "f": f_val,
            "f1": us_data.get("f1", f_val),
            "f2": us_data.get("f2", f_val),
            "l": us_data.get("l", 0),
            "r": us_data.get("r", 0),
            "b": us_data.get("b", 0)
        }
        if show_monitor_display and warmup_frame is not None:
            cd_img = warmup_frame.copy()
            cv2.putText(cd_img, f"STARTING IN {c} SECONDS...", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            cv2.imshow(window_name, cd_img)
            cv2.waitKey(1)
        time.sleep(1.0)

    print(f"[START SNAPSHOT] Home Baseline Position Recorded: {start_snapshot}")

    # ------------------------------------------------------------------------
    # Phase 2: Start Active Driving
    # ------------------------------------------------------------------------
    print("[START] Driving FORWARD (Phase 2: Ultrasonics off for zero-lag vision)!")
    serial_ctrl.send_command("AUTO_US_OFF")
    serial_ctrl.send_command("FORWARD")

    ROI1 = [20, 170, 240, 220]   # Left wall ROI
    ROI2 = [400, 170, 620, 220]  # Right wall ROI
    ROI3 = [200, 300, 440, 350]  # Ground indicator line ROI

    t = 0                  # Completed turn count (3 laps x 4 turns = 12)
    turnDir = forced_dir   # Track direction ("left", "right", or "none")
    lDetected = False
    isTurning = False
    turnStartTime = 0
    lineLockoutUntil = 0   # 3.5s line detection lockout timer
    turnCooldownUntil = 0  # 3.5s turn trigger cooldown timer
    lockoutDuration = 3.5  # Exactly 3.5 seconds lockout

    normalSpeed = 245      # Full straightaway speed (96% PWM)
    turnSpeed = 195        # Reduced turn & cornering speed to prevent drifting when steering > 30 deg or max 40 deg
    returnSpeed = 230      # Controlled approach speed for pin-point finish stopping (230 PWM)

    minTurnDuration = 0.8  # Minimum arc turn time before checking wall re-acquisition (0.8s)
    maxTurnDuration = 2.2  # Safety maximum turn time cap (2.2s)
    wallReacquireArea = 600 # Area threshold to confirm single wall in narrow FOV view
    turnThresh = 200       # Area threshold below which wall end is detected

    is_returning_home = False          # True once 12th (final) corner exit is confirmed
    corner12_exit_time = 0             # Timestamp when 12th turn exit was confirmed
    home_stop_initiated = False        # True once final stop sequence is committed

    MIN_CLEAR_OF_CORNER_TIME = 0.5     # Min time after turn-12 exit before allowing line/sensor stop (0.5s)
    FRONT_WALL_HARD_STOP_CM = 25.0     # Hard safety ceiling: stop if front wall <= 25cm
    HOME_ABSOLUTE_TIMEOUT = 4.0        # Absolute maximum timeout cap since turn-12 exit

    last_steer_angle = None
    last_drive_speed = None
    last_cmd_time = 0

    try:
        while True:
            img = camera.capture_array()
            if img is None:
                time.sleep(0.01)
                continue

            currTime = time.time()
            img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

            cListLeft = find_black_wall_contours(img, ROI1)
            cListRight = find_black_wall_contours(img, ROI2)
            cListOrange = find_contours(img_lab, rOrange, ROI3)
            cListBlue = find_contours(img_lab, rBlue, ROI3)

            leftArea = max_contour(cListLeft, ROI1)[0]
            rightArea = max_contour(cListRight, ROI2)[0]
            orangeArea = max_contour(cListOrange, ROI3)[0]
            blueArea = max_contour(cListBlue, ROI3)[0]

            us_data = serial_ctrl.get_us_data() if is_returning_home else {}
            f_us = us_data.get("f", 0)
            l_us = us_data.get("l", 0)
            r_us = us_data.get("r", 0)
            b_us = us_data.get("b", 0)

            # -------------------------------------------------------------
            # 1. PERMANENT FIRST-COLOR DIRECTION LOCK & MARKER DETECTION
            # -------------------------------------------------------------
            if t < 12 and not isTurning and not is_returning_home and currTime >= lineLockoutUntil:
                if turnDir == "none":
                    if orangeArea > 100 and orangeArea > blueArea:
                        turnDir = "right"
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[FIRST-COLOR LOCK] First Line Detected: ORANGE ({orangeArea} px) -> Permanently Locking Direction to RIGHT!")
                    elif blueArea > 100 and blueArea > orangeArea:
                        turnDir = "left"
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[FIRST-COLOR LOCK] First Line Detected: BLUE ({blueArea} px) -> Permanently Locking Direction to LEFT!")
                
                elif turnDir == "right":
                    if orangeArea > 100:
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[LOCKED MARKER] Detected ORANGE Line ({orangeArea} px) -> Track Dir = RIGHT (3.5s Line Lockout)")
                
                elif turnDir == "left":
                    if blueArea > 100:
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[LOCKED MARKER] Detected BLUE Line ({blueArea} px) -> Track Dir = LEFT (3.5s Line Lockout)")

            # -------------------------------------------------------------
            # 2. HYBRID CORNER TURN & DYNAMIC VISION EXIT (CORRECTED: 60=LEFT, 140=RIGHT)
            # -------------------------------------------------------------
            if isTurning and not is_returning_home:
                # FIX: 60 deg is LEFT turn, 140 deg is RIGHT turn (matching ESP32 hardware steering!)
                targetTurnAngle = 60 if turnDir == "left" else 140
                
                if (currTime - last_cmd_time) >= 0.1 or last_drive_speed != turnSpeed:
                    serial_ctrl.send_command(f"DRIVE:{turnSpeed}:{targetTurnAngle}")
                    last_cmd_time = currTime
                    last_drive_speed = turnSpeed
                    last_steer_angle = targetTurnAngle

                turnElapsed = currTime - turnStartTime

                # DYNAMIC TURN EXIT CONDITION:
                newWallAcquired = (turnElapsed >= minTurnDuration) and (leftArea >= wallReacquireArea or rightArea >= wallReacquireArea)
                maxTimeoutReached = (turnElapsed >= maxTurnDuration)

                if newWallAcquired or maxTimeoutReached:
                    isTurning = False
                    turnCooldownUntil = currTime + lockoutDuration
                    lineLockoutUntil = currTime + lockoutDuration
                    exit_reason = "WALL_REACQUIRED" if newWallAcquired else "MAX_TIMEOUT"
                    print(f"[NAV EVENT] Turn {t}/12 ({turnDir.upper()}) EXITED via {exit_reason} in {round(turnElapsed, 2)}s!")

                    if t >= 12:
                        is_returning_home = True
                        corner12_exit_time = currTime
                        serial_ctrl.send_command("AUTO_US_ON")
                        print("=" * 65)
                        print(f"[PHASE 3] Final corner cleared! Approaching Start/Finish Line at returnSpeed {returnSpeed}...")
                        print("[PHASE 3] Ultrasonics & Finish Line Scanner Active.")
                        print("=" * 65)

            elif (t < 12) and currTime >= turnCooldownUntil and not is_returning_home:
                wallDropDetected = (leftArea <= turnThresh and rightArea <= turnThresh) or \
                                   (turnDir == "left" and leftArea <= turnThresh) or \
                                   (turnDir == "right" and rightArea <= turnThresh)

                if (lDetected or forced_dir != "none") and wallDropDetected:
                    # FIX: 60 deg is LEFT turn, 140 deg is RIGHT turn!
                    targetTurnAngle = 60 if turnDir == "left" else 140
                    t += 1
                    print(f"[NAV EVENT] Marker Seen + Wall Drop! (L:{leftArea} R:{rightArea}) -> Triggering Turn ({t}/12) angle={targetTurnAngle}...")
                    serial_ctrl.send_command(f"DRIVE:{turnSpeed}:{targetTurnAngle}")
                    last_cmd_time = currTime
                    last_drive_speed = turnSpeed
                    last_steer_angle = targetTurnAngle
                    isTurning = True
                    turnStartTime = currTime
                    lDetected = False
                    turnCooldownUntil = currTime + maxTurnDuration + lockoutDuration

            # -------------------------------------------------------------
            # 3. STRAIGHTAWAY WALL AVOIDANCE & DYNAMIC SPEED CONTROL (Laps 1-3)
            # -------------------------------------------------------------
            if not isTurning and not is_returning_home:
                serial_ctrl.send_command("AUTO_US_OFF")

                aDiff = rightArea - leftArea
                steer_angle = int(100 - (aDiff * 0.02))
                steer_angle = max(60, min(140, steer_angle))

                steerDeflection = abs(steer_angle - 100)
                currentSpeed = turnSpeed if steerDeflection > 30 else normalSpeed

                angle_changed = last_steer_angle is None or abs(steer_angle - last_steer_angle) >= 2
                speed_changed = last_drive_speed != currentSpeed
                time_elapsed = (currTime - last_cmd_time) >= 0.1

                if angle_changed or speed_changed or time_elapsed:
                    serial_ctrl.send_command(f"DRIVE:{currentSpeed}:{steer_angle}")
                    last_steer_angle = steer_angle
                    last_drive_speed = currentSpeed
                    last_cmd_time = currTime

            # -------------------------------------------------------------
            # 4. Phase 3: PRECISION STARTING SECTION STOPPING ENGINE
            # -------------------------------------------------------------
            if is_returning_home and not home_stop_initiated:
                elapsed_since_corner = currTime - corner12_exit_time

                aDiff = rightArea - leftArea
                gentle_steer = int(100 - (aDiff * 0.01))
                gentle_steer = max(80, min(120, gentle_steer))

                reasons = []

                line_marker_detected = (
                    (turnDir == "right" and orangeArea > 150) or
                    (turnDir == "left" and blueArea > 150) or
                    (turnDir == "none" and (orangeArea > 150 or blueArea > 150))
                )
                if elapsed_since_corner >= MIN_CLEAR_OF_CORNER_TIME and line_marker_detected:
                    reasons.append(f"START_FINISH_LINE_MARKER(O:{orangeArea},B:{blueArea})")

                init_b = start_snapshot.get("b", 0)
                init_f = start_snapshot.get("f", 0)
                b_match = (init_b > 0 and b_us > 0 and abs(b_us - init_b) <= 4.0)
                f_match = (init_f > 0 and f_us > 0 and abs(f_us - init_f) <= 4.0)
                if elapsed_since_corner >= MIN_CLEAR_OF_CORNER_TIME and (b_match or f_match):
                    reasons.append(f"BASELINE_SENSOR_MATCH(F:{f_us}/{init_f},B:{b_us}/{init_b})")

                if f_us > 0 and f_us <= FRONT_WALL_HARD_STOP_CM:
                    reasons.append(f"FRONT_WALL_PROXIMITY({f_us}cm)")

                if elapsed_since_corner >= HOME_ABSOLUTE_TIMEOUT:
                    reasons.append("SAFETY_TIMEOUT_CAP")

                should_stop_now = len(reasons) > 0

                if should_stop_now:
                    home_stop_initiated = True
                    print("=" * 65)
                    print(f"[FINISH PRECISION STOP] Executing Active Electronic Reverse Brake Pulse!")
                    print(f"[FINISH METRICS] Reasons: {reasons}")
                    print(f"[FINISH METRICS] Elapsed since Turn 12 exit: {round(elapsed_since_corner, 2)}s")
                    print(f"[SENSORS] Current F:{f_us} L:{l_us} R:{r_us} B:{b_us} | Baseline: {start_snapshot}")
                    print("=" * 65)

                    serial_ctrl.send_command("STOP")
                    serial_ctrl.send_command("DRIVE:-180:100")
                    time.sleep(0.08)
                    serial_ctrl.send_command("STOP")
                    last_drive_speed = 0
                    time.sleep(0.5)
                    break
                else:
                    if (currTime - last_cmd_time) >= 0.1 or last_drive_speed != returnSpeed:
                        serial_ctrl.send_command(f"DRIVE:{returnSpeed}:{gentle_steer}")
                        last_cmd_time = currTime
                        last_drive_speed = returnSpeed
                        last_steer_angle = gentle_steer

            elif home_stop_initiated:
                serial_ctrl.send_command("STOP")
                last_drive_speed = 0
                time.sleep(0.5)
                break

            img_disp = img.copy()
            draw_roi(img_disp, ROI1, (0, 255, 255), 2)
            draw_roi(img_disp, ROI2, (0, 255, 255), 2)
            draw_roi(img_disp, ROI3, (255, 255, 0), 2)
            draw_offset_contours(img_disp, cListLeft, ROI1, (0, 255, 0), 2)
            draw_offset_contours(img_disp, cListRight, ROI2, (0, 255, 0), 2)
            draw_offset_contours(img_disp, cListOrange, ROI3, (0, 165, 255), 2)
            draw_offset_contours(img_disp, cListBlue, ROI3, (255, 0, 0), 2)

            cam_type = "WEBCAM" if camera.is_webcam else "PICAM2"
            lock_rem = max(0.0, round(lineLockoutUntil - currTime, 1))
            lock_str = f"LOCKED({lock_rem}s)" if lock_rem > 0 else "READY"
            
            if is_returning_home:
                state_str = f"RETURN_TO_HOME ({returnSpeed})"
            elif isTurning:
                t_ela = round(currTime - turnStartTime, 1)
                state_str = f"TURNING ({turnDir.upper()} {t_ela}s)"
            else:
                state_str = f"VISION_WALLS ({turnDir.upper()})"
            
            active_speed = last_drive_speed if last_drive_speed is not None else normalSpeed
            telemetry_text = f"Cam:{cam_type} | State:{state_str} | Speed:{active_speed} | Turns:{t}/12"
            wall_text = f"Walls -> Left:{leftArea}px | Right:{rightArea}px | LineLock:{lock_str}"
            us_text = f"US Sensors -> F:{f_us}cm | L:{l_us}cm | R:{r_us}cm | B:{b_us}cm"

            cv2.putText(img_disp, telemetry_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 204), 2)
            cv2.putText(img_disp, wall_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            cv2.putText(img_disp, us_text, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            if show_monitor_display:
                cv2.imshow(window_name, img_disp)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("[USER INTERRUPT] Stopping bot from monitor GUI...")
                    serial_ctrl.send_command("STOP")
                    break
                elif key == ord('l'):
                    turnDir = "left"
                    print("[KEYBOARD OVERRIDE] Direction set to LEFT")
                elif key == ord('r'):
                    turnDir = "right"
                    print("[KEYBOARD OVERRIDE] Direction set to RIGHT")

            display_variables({
                "Camera Type": cam_type,
                "State": state_str,
                "Track Dir": turnDir,
                "Speed (PWM)": active_speed,
                "Turn Count": f"{t}/12",
                "Line Lockout": lock_str,
                "Line Detected": lDetected,
                "Left Wall Area (px)": leftArea,
                "Right Wall Area (px)": rightArea,
                "US Front (cm)": f_us,
                "US Left (cm)": l_us,
                "US Right (cm)": r_us,
                "US Back (cm)": b_us,
                "Home Baseline": start_snapshot
            })

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[SAFETY] Keyboard Interrupt. Halting bot...")
    finally:
        serial_ctrl.send_command("STOP")
        camera.stop()
        time.sleep(0.1)
        serial_ctrl.disconnect()
        if show_monitor_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
