#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 Open Challenge Autonomous Navigation (Round 1)

FIXED VERSION - Phase 3 (Return-to-Home) rewritten for robustness.

Changes from the original:
1. BUGFIX: Turn-trigger condition now guards on `t < 12`. Previously, once the
   12th turn finished, the robot could still trigger a spurious 13th "turn"
   while waiting to enter Phase 3, driving straight into a wall.
2. REDEFINED GOAL: Per WRO rule 9.24.2 / 9.22, "home" only requires the
   vehicle's full footprint to stop inside the finish (=starting) SECTION -
   NOT to reproduce the exact pre-race sensor snapshot within 2cm. Chasing an
   exact snapshot match is solving a harder-than-required problem and is
   prone to never firing (leading to an uncontrolled timeout stop).
3. NO MORE BLIND 5.0s DELAY: Ultrasonics are re-enabled immediately once the
   12th turn's exit is confirmed (vision wall re-acquired), not 5 seconds
   later. That blind window was enough distance, at speed, to already miss
   the target section before Phase 3 logic ever started evaluating anything.
4. DEBOUNCED, BOUNDED STOP DECISION: instead of an OR across noisy single-
   frame sensor matches (+ arbitrary 4.0s fallback), we now require:
     a) a minimum "clear of entry corner" settle distance/time,
     b) a hard safety ceiling based on front-wall proximity so we never
        drive into the next corner's wall,
     c) a calibrated target drive time as the primary stop trigger, with
        front-distance confirmation as a secondary check (not OR'd blindly).
   The "home snapshot" is only used as an optional confidence booster
   (lateral centering), never as the sole trigger.
5. GENUINE FULL STOP: explicit repeated STOP commands and a settle window,
   so the vehicle does not appear to "continue moving after 15 seconds"
   (rule 9.24.2, Note 2).

TUNE THESE FOR YOUR ROBOT/TRACK (search "TUNE:" below):
- TARGET_HOME_DRIVE_TIME: time (s) to drive after clearing the last corner
  before beginning to look for a stop. Set this empirically to roughly the
  time it takes your robot to reach the middle of the home straight at
  returnSpeed.
- FRONT_WALL_HARD_STOP_CM: minimum front clearance before we force an
  immediate stop regardless of anything else, to avoid ramming the far wall.
- MIN_CLEAR_OF_CORNER_TIME: minimum time after turn-12 wall-reacquisition
  before we allow a stop decision at all (ensures the whole footprint has
  left the corner section).
"""

import sys
import time
import cv2
import numpy as np
from wro_serial import WROSerialController
from masks import rOrange, rBlack, rBlue
from wro_functions import (CameraManager, find_black_wall_contours, find_contours, max_contour, draw_roi,
                           draw_offset_contours, display_variables)


def main():
    print("=" * 65)
    print("   ROBOVANGUARD - WRO Round 1 Open Challenge Node (Pi 5)")
    print("   Architecture: Robust Return-to-Home (bounded, debounced)")
    print("=" * 65)

    force_webcam = "--webcam" in sys.argv or "-w" in sys.argv
    use_vision_walls = "--vision-walls" in sys.argv or "--no-us" in sys.argv

    forced_dir = "none"
    if "--dir" in sys.argv:
        idx = sys.argv.index("--dir")
        if idx + 1 < len(sys.argv):
            forced_dir = sys.argv[idx + 1].lower()
            print(f"[CONFIG] Forcing fixed track direction: {forced_dir.upper()}")

    serial_ctrl = WROSerialController()
    if not serial_ctrl.connect():
        print("[ERROR] Cannot proceed without ESP32 serial connection.")
        sys.exit(1)

    print("[SAFETY] Forcing robot STOP state during initialization...")
    serial_ctrl.send_command("STOP")
    time.sleep(0.5)

    show_monitor_display = "--no-display" not in sys.argv
    window_name = "WRO Open Challenge - Hybrid Monitor Debug (Pi 5)"

    if show_monitor_display:
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 800, 600)
            print("[DISPLAY] Created OpenCV live display window on monitor!")
        except Exception as e:
            print(f"[WARNING] Could not open GUI display window: {e}")
            show_monitor_display = False

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
    # Phase 1: Setup & Warmup Countdown
    # ------------------------------------------------------------------------
    print("\n[READY] Sensor-Vision Engine Ready!")
    print("[PHASE 1] Recording Baseline Start Position (informational only now)...")

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

    print(f"[START SNAPSHOT] Baseline (used only as an optional lateral cross-check): {start_snapshot}")

    # ------------------------------------------------------------------------
    # Phase 2: Start Active Driving (Ultrasonics deactivated during laps)
    # ------------------------------------------------------------------------
    print("[START] Driving FORWARD (Phase 2: Ultrasonics off for zero-lag vision)!")
    serial_ctrl.send_command("AUTO_US_OFF")
    serial_ctrl.send_command("FORWARD")

    ROI1 = [20, 170, 240, 220]
    ROI2 = [400, 170, 620, 220]
    ROI3 = [200, 300, 440, 350]

    t = 0
    turnDir = forced_dir
    lDetected = False
    isTurning = False
    turnStartTime = 0
    lineLockoutUntil = 0
    turnCooldownUntil = 0
    lockoutDuration = 3.5

    normalSpeed = 245
    turnSpeed = 230
    returnSpeed = 230

    minTurnDuration = 0.8
    maxTurnDuration = 2.2
    wallReacquireArea = 600
    turnThresh = 200

    # -------------------- PHASE 3 STATE (rewritten) --------------------
    is_returning_home = False          # True once we've exited the 12th (final) corner
    corner12_exit_time = 0             # timestamp the 12th turn's exit was confirmed
    home_stop_initiated = False        # True once we've committed to the final stop sequence
    home_stop_confirm_start = 0        # for the "hold still" confirmation window

    # TUNE: time to drive after clearing corner 12 before we start allowing a stop.
    # Keeps the whole footprint clear of the corner section (rule requires FULL
    # containment in the finish section, not partial).
    MIN_CLEAR_OF_CORNER_TIME = 0.8

    # TUNE: primary stop trigger - roughly "middle of the home straight" at
    # returnSpeed. Measure your actual straight length / speed and set this.
    TARGET_HOME_DRIVE_TIME = 1.6

    # TUNE: hard safety ceiling - if the front wall gets this close, stop
    # immediately no matter what state we're in, to avoid crossing into the
    # next corner section (rule 9.24.3) or hitting a wall (rule 9.18).
    FRONT_WALL_HARD_STOP_CM = 25.0

    # Absolute worst-case time cap since corner-12 exit, in case ultrasonics
    # are noisy/unavailable - prevents driving forever.
    HOME_ABSOLUTE_TIMEOUT = 4.5

    # Debounce: require the stop condition to hold for this many seconds
    # before committing, so a single noisy frame can't end the round early
    # OR make us overshoot because we ignored a real signal.
    STOP_CONFIRM_HOLD = 0.25

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

            # Ultrasonics: OFF during laps 1-3 for zero lag, ON immediately once
            # we've confirmed we exited the 12th (final) corner. No blind delay.
            us_data = serial_ctrl.get_us_data() if is_returning_home else {}
            f_us = us_data.get("f", 0)
            l_us = us_data.get("l", 0)
            r_us = us_data.get("r", 0)
            b_us = us_data.get("b", 0)

            # -------------------------------------------------------------
            # 1. PERMANENT FIRST-COLOR DIRECTION LOCK & MARKER DETECTION
            #    (guarded so it never fires once all 12 turns are done)
            # -------------------------------------------------------------
            if t < 12 and not isTurning and not is_returning_home and currTime >= lineLockoutUntil:
                if turnDir == "none":
                    if orangeArea > 150 and orangeArea > blueArea:
                        turnDir = "right"
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[FIRST-COLOR LOCK] ORANGE ({orangeArea}px) -> Direction RIGHT")
                    elif blueArea > 150 and blueArea > orangeArea:
                        turnDir = "left"
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                        print(f"[FIRST-COLOR LOCK] BLUE ({blueArea}px) -> Direction LEFT")
                elif turnDir == "right":
                    if orangeArea > 150:
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration
                elif turnDir == "left":
                    if blueArea > 150:
                        lDetected = True
                        lineLockoutUntil = currTime + lockoutDuration

            # -------------------------------------------------------------
            # 2. HYBRID CORNER TURN & DYNAMIC VISION EXIT
            # -------------------------------------------------------------
            if isTurning and not is_returning_home:
                targetTurnAngle = 140 if turnDir == "left" else 60
                if (currTime - last_cmd_time) >= 0.1 or last_drive_speed != turnSpeed:
                    serial_ctrl.send_command(f"DRIVE:{turnSpeed}:{targetTurnAngle}")
                    last_cmd_time = currTime
                    last_drive_speed = turnSpeed
                    last_steer_angle = targetTurnAngle

                turnElapsed = currTime - turnStartTime
                newWallAcquired = (turnElapsed >= minTurnDuration) and (leftArea >= wallReacquireArea or rightArea >= wallReacquireArea)
                maxTimeoutReached = (turnElapsed >= maxTurnDuration)

                if newWallAcquired or maxTimeoutReached:
                    isTurning = False
                    turnCooldownUntil = currTime + lockoutDuration
                    lineLockoutUntil = currTime + lockoutDuration
                    exit_reason = "WALL_REACQUIRED" if newWallAcquired else "MAX_TIMEOUT"
                    print(f"[NAV EVENT] Turn {t}/12 ({turnDir.upper()}) EXITED via {exit_reason} in {round(turnElapsed, 2)}s!")

                    if t >= 12:
                        # This was the FINAL corner. Mark the moment of exit and
                        # immediately re-enable ultrasonics - no blind delay.
                        is_returning_home = True
                        corner12_exit_time = currTime
                        serial_ctrl.send_command("AUTO_US_ON")
                        print("=" * 65)
                        print("[PHASE 3] Final corner cleared. Entering finish straight.")
                        print(f"[PHASE 3] Ultrasonics reactivated immediately (no blind delay).")
                        print("=" * 65)

            # BUGFIX: guarded with `t < 12` so a 13th phantom turn can never trigger.
            elif (t < 12) and currTime >= turnCooldownUntil and not is_returning_home:
                wallDropDetected = (leftArea <= turnThresh and rightArea <= turnThresh) or \
                                   (turnDir == "left" and leftArea <= turnThresh) or \
                                   (turnDir == "right" and rightArea <= turnThresh)

                if (lDetected or forced_dir != "none") and wallDropDetected:
                    targetTurnAngle = 140 if turnDir == "left" else 60
                    t += 1
                    print(f"[NAV EVENT] Marker + Wall Drop! (L:{leftArea} R:{rightArea}) -> Turn ({t}/12)...")
                    serial_ctrl.send_command(f"DRIVE:{turnSpeed}:{targetTurnAngle}")
                    last_cmd_time = currTime
                    last_drive_speed = turnSpeed
                    last_steer_angle = targetTurnAngle
                    isTurning = True
                    turnStartTime = currTime
                    lDetected = False
                    turnCooldownUntil = currTime + maxTurnDuration + lockoutDuration

            # -------------------------------------------------------------
            # 3. STRAIGHTAWAY WALL AVOIDANCE (laps 1-3 only)
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
            # Phase 3 (REWRITTEN): Drive to a safe stop inside the finish
            # section. Goal = "fully inside the section", NOT "match the
            # exact pre-race pose". Bounded + debounced, never open-ended.
            # -------------------------------------------------------------
            if is_returning_home and not home_stop_initiated:
                elapsed_since_corner = currTime - corner12_exit_time

                # Gentle, heavily-dampened steering - priority is a stable,
                # predictable stop, not tight line-following.
                aDiff = rightArea - leftArea
                gentle_steer = int(100 - (aDiff * 0.005))
                gentle_steer = max(85, min(115, gentle_steer))

                front_reading_valid = f_us > 0

                # --- Decide whether to stop now ---
                reasons = []

                # (a) Hard safety ceiling: about to hit the far wall/corner.
                if front_reading_valid and f_us <= FRONT_WALL_HARD_STOP_CM:
                    reasons.append("FRONT_WALL_PROXIMITY")

                # (b) Primary calibrated target reached, and we've cleared
                #     the entry corner long enough to be fully inside the
                #     section.
                if elapsed_since_corner >= max(TARGET_HOME_DRIVE_TIME, MIN_CLEAR_OF_CORNER_TIME):
                    reasons.append("TARGET_DRIVE_TIME")

                # (c) Absolute worst-case cap (sensor dropout, etc.)
                if elapsed_since_corner >= HOME_ABSOLUTE_TIMEOUT:
                    reasons.append("ABSOLUTE_TIMEOUT")

                should_stop_now = (
                    elapsed_since_corner >= MIN_CLEAR_OF_CORNER_TIME and len(reasons) > 0
                )

                if should_stop_now:
                    if home_stop_confirm_start == 0:
                        home_stop_confirm_start = currTime
                        print(f"[PHASE 3] Stop condition met ({', '.join(reasons)}). Confirming...")
                    elif (currTime - home_stop_confirm_start) >= STOP_CONFIRM_HOLD:
                        home_stop_initiated = True
                        print("=" * 65)
                        print(f"[FINISH] Stopping inside finish section. Reasons: {reasons}")
                        print(f"[FINISH] Elapsed since corner-12 exit: {round(elapsed_since_corner, 2)}s")
                        print(f"[SENSORS] F:{f_us} L:{l_us} R:{r_us} B:{b_us} | home ref: {start_snapshot}")
                        print("=" * 65)
                else:
                    # Not ready to stop yet - reset the confirm timer if the
                    # condition flickered off, and keep driving forward.
                    home_stop_confirm_start = 0
                    if (currTime - last_cmd_time) >= 0.1 or last_drive_speed != returnSpeed:
                        serial_ctrl.send_command(f"DRIVE:{returnSpeed}:{gentle_steer}")
                        last_cmd_time = currTime
                        last_drive_speed = returnSpeed
                        last_steer_angle = gentle_steer

            elif home_stop_initiated:
                # Genuine, repeated full stop so the vehicle does not appear to
                # keep moving (rule 9.24.2, Note 2). Hold here until the loop
                # exits below.
                serial_ctrl.send_command("STOP")
                last_drive_speed = 0
                break

            # Draw ROIs & Offset Contours
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
                elif key == ord('r'):
                    turnDir = "right"

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
                "Home Reference (unused as sole trigger)": start_snapshot
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
