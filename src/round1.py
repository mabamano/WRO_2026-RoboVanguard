#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers
Round 1: Open Track Challenge Code

Description:
    Autonomous navigation algorithm for Round 1 (Open Track).
    Uses Raspberry Pi 5 + Pi Camera to capture track video, perform Lab color space
    filtering to detect black lane walls, compute PD steering angle corrections based on
    contour area differences, recognize direction indicator lines (Orange/Blue), and track lap counts.
    Transmits motion commands ('f','l','r','s') via serial UART to the ESP32 RoboGuard motor controller.

Team: ROBOVANGUARD (Ramco Institute of Technology)
"""

import sys
import time
import numpy as np
import cv2
import serial

# Try importing Picamera2; fallback to OpenCV VideoCapture if unavailable
HAS_PICAM2 = False
try:
    from picamera2 import Picamera2
    from picamera2.previews import Preview
    HAS_PICAM2 = True
except ImportError:
    HAS_PICAM2 = False

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
SEND_INTERVAL_SEC = 0.05
APPEND_NEWLINE = True

CMD_RIGHT = 'r'
CMD_LEFT  = 'l'
CMD_FWD   = 'f'
CMD_STOP  = 's'

FRAME_WIDTH, FRAME_HEIGHT = 640, 480
TARGET_LAPS = 3

# Regions of Interest [x1, y1, x2, y2]
ROI_LEFT  = [20, 170, 240, 220]   # Left wall detection
ROI_RIGHT = [400, 170, 620, 220]  # Right wall detection
ROI_INDIC = [200, 300, 440, 350]  # Orange/Blue turn indicator detection

# Steering & PD Parameters
KP = 0.02
KD = 0.006
STRAIGHT_ANGLE = 87
TURN_THRESH = 150
EXIT_THRESH = 1500
ANGLE_THRESH = 4.0
FAILSAFE_MIN_BLACK_AREA = 50

# Lab Color Space Mask Thresholds [L, a, b]
LAB_LOWER_BLACK  = np.array([0, 0, 0], np.uint8)
LAB_UPPER_BLACK  = np.array([70, 255, 255], np.uint8)
LAB_LOWER_ORANGE = np.array([40, 150, 150], np.uint8)
LAB_UPPER_ORANGE = np.array([255, 205, 255], np.uint8)
LAB_LOWER_BLUE   = np.array([20, 110, 0], np.uint8)
LAB_UPPER_BLUE   = np.array([255, 170, 110], np.uint8)

MIN_AREA_LANE  = 60
MIN_AREA_INDIC = 80
INDIC_THRESH   = 100

# ==============================================================================
# SERIAL COMMUNICATION INTERFACE
# ==============================================================================
class SerialController:
    """Manages high-speed serial UART communication with ESP32 motor controller."""
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD):
        self.ser = None
        self.last_cmd = None
        self.last_send_time = 0.0
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # Allow ESP32 reset after serial connection
            print(f"[INFO] Serial interface opened on {port} @ {baud} baud.")
        except Exception as err:
            print(f"[WARN] Serial port could not be opened: {err}")

    def send_cmd(self, cmd):
        """Send movement command character if state changed or timeout reached."""
        now = time.time()
        if (cmd != self.last_cmd) or (now - self.last_send_time >= SEND_INTERVAL_SEC):
            payload = (cmd + ("\n" if APPEND_NEWLINE else "")).encode('ascii')
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(payload)
                except Exception as e:
                    print(f"[ERROR] Serial write error: {e}")
            print(f"[TX] Command: {cmd}")
            self.last_cmd = cmd
            self.last_send_time = now

    def close(self):
        """Send stop command and close serial connection."""
        if self.ser and self.ser.is_open:
            try:
                self.send_cmd(CMD_STOP)
                self.ser.close()
            except Exception:
                pass

# ==============================================================================
# IMAGE PROCESSING HELPERS
# ==============================================================================
def morphology_clean(mask, ksize=5, iterations=1):
    """Applies morphological close operation to fill noise gaps in binary mask."""
    kernel = np.ones((ksize, ksize), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)

def find_contours_lab(img_lab_roi, lower, upper, min_area):
    """Thresholds ROI in Lab space and filters contours exceeding min_area."""
    mask = cv2.inRange(img_lab_roi, lower, upper)
    mask = cv2.GaussianBlur(mask, (7, 7), 0)
    mask = morphology_clean(mask, 5, 1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]

def max_contour_area(contours):
    """Returns the maximum area among a list of contours, or 0 if empty."""
    return int(max((cv2.contourArea(c) for c in contours), default=0))

def slice_roi(img, roi):
    """Slices image array according to ROI coordinates [x1, y1, x2, y2]."""
    x1, y1, x2, y2 = roi
    return img[y1:y2, x1:x2]

def draw_roi(frame, roi, color, thick=2):
    """Draws ROI boundary box on visualization frame."""
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)

def draw_offset_contours(frame, contours, roi, color, thick=2):
    """Draws contours offset back to full frame coordinates."""
    if not contours:
        return
    x1, y1, _, _ = roi
    offset = np.array([[x1, y1]], dtype=np.int32)
    shifted = [cnt + offset for cnt in contours]
    cv2.drawContours(frame, shifted, -1, color, thick)

# ==============================================================================
# MAIN NAVIGATION ROUTINE
# ==============================================================================
def run_round1():
    print("=" * 60)
    print("🤖 ROBOVANGUARD - Starting Round 1 (Open Track Challenge)")
    print("=" * 60)

    serial_ctrl = SerialController()

    # Initialize Camera
    picam2 = None
    cap = None
    if HAS_PICAM2:
        for attempt in range(10):
            try:
                picam2 = Picamera2()
                picam2.preview_configuration.main.size = (FRAME_WIDTH, FRAME_HEIGHT)
                picam2.preview_configuration.main.format = "BGR888"
                picam2.preview_configuration.align()
                picam2.configure("preview")
                picam2.start()
                print(f"[INFO] Picamera2 initialized on attempt {attempt+1}")
                break
            except Exception as e:
                print(f"[WARN] Camera init retry ({attempt+1}): {e}")
                time.sleep(0.5)
        else:
            print("[ERROR] Failed to start Picamera2. Falling back to OpenCV VideoCapture...")
            HAS_PICAM2 = False

    if not HAS_PICAM2:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # State variables
    l_turn = False
    r_turn = False
    laps_completed = 0
    turn_dir = "none"
    line_detected = False
    prev_diff = 0.0
    prev_angle = STRAIGHT_ANGLE

    cv2.namedWindow("ROBOVANGUARD - Round 1", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ROBOVANGUARD - Round 1", FRAME_WIDTH, FRAME_HEIGHT)

    try:
        while True:
            # Capture frame
            if HAS_PICAM2 and picam2 is not None:
                frame_rgb = picam2.capture_array()
                if frame_rgb is None or frame_rgb.size == 0:
                    continue
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("[WARN] Unable to capture frame from webcam.")
                    time.sleep(0.03)
                    continue

            # Convert frame to Lab color space
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
            lab = cv2.GaussianBlur(lab, (7, 7), 0)

            # Extract ROIs
            roi_left_lab  = slice_roi(lab, ROI_LEFT)
            roi_right_lab = slice_roi(lab, ROI_RIGHT)
            roi_ind_lab   = slice_roi(lab, ROI_INDIC)

            # Find contours
            c_left   = find_contours_lab(roi_left_lab,  LAB_LOWER_BLACK,  LAB_UPPER_BLACK,  MIN_AREA_LANE)
            c_right  = find_contours_lab(roi_right_lab, LAB_LOWER_BLACK,  LAB_UPPER_BLACK,  MIN_AREA_LANE)
            c_orange = find_contours_lab(roi_ind_lab,   LAB_LOWER_ORANGE, LAB_UPPER_ORANGE, MIN_AREA_INDIC)
            c_blue   = find_contours_lab(roi_ind_lab,   LAB_LOWER_BLUE,   LAB_UPPER_BLUE,   MIN_AREA_INDIC)

            left_area  = max_contour_area(c_left)
            right_area = max_contour_area(c_right)
            orange_area = max_contour_area(c_orange)
            blue_area   = max_contour_area(c_blue)

            # Determine turn indicator line (Orange = Clockwise, Blue = Counter-Clockwise)
            if orange_area > INDIC_THRESH:
                line_detected = True
                if turn_dir == "none":
                    turn_dir = "right (CW)"
            elif blue_area > INDIC_THRESH:
                line_detected = True
                if turn_dir == "none":
                    turn_dir = "left (CCW)"

            # Draw visual HUD overlays
            draw_roi(frame, ROI_LEFT,  (0, 255, 255), 2)
            draw_roi(frame, ROI_RIGHT, (0, 255, 255), 2)
            draw_roi(frame, ROI_INDIC, (255, 255, 0), 2)
            draw_offset_contours(frame, c_left,   ROI_LEFT,  (0, 255, 0), 2)
            draw_offset_contours(frame, c_right,  ROI_RIGHT, (0, 255, 0), 2)
            draw_offset_contours(frame, c_orange, ROI_INDIC, (0, 165, 255), 2)
            draw_offset_contours(frame, c_blue,   ROI_INDIC, (255, 0, 0), 2)

            # Proportional-Derivative (PD) Steering Calculation
            area_diff = float(right_area - left_area)
            angle = STRAIGHT_ANGLE + KP * area_diff + KD * (area_diff - prev_diff)

            # Wall detection trigger
            if left_area <= TURN_THRESH and not r_turn:
                l_turn = True
            elif right_area <= TURN_THRESH and not l_turn:
                r_turn = True

            # Exit corner condition & lap counter logic
            if (l_turn or r_turn) and ((r_turn and right_area > EXIT_THRESH) or (l_turn and left_area > EXIT_THRESH)):
                l_turn = r_turn = False
                prev_diff = 0.0
                if line_detected:
                    laps_completed += 1
                    print(f"[SUCCESS] Lap {laps_completed} / {TARGET_LAPS} completed!")
                    line_detected = False
                turn_dir = "none"

            # Command arbitration
            if left_area < FAILSAFE_MIN_BLACK_AREA and right_area < FAILSAFE_MIN_BLACK_AREA:
                cmd = CMD_STOP
                l_turn = r_turn = False
            else:
                if l_turn:
                    cmd = CMD_LEFT
                elif r_turn:
                    cmd = CMD_RIGHT
                else:
                    delta = angle - STRAIGHT_ANGLE
                    if delta >= ANGLE_THRESH:
                        cmd = CMD_LEFT
                    elif delta <= -ANGLE_THRESH:
                        cmd = CMD_RIGHT
                    else:
                        cmd = CMD_FWD

            # Check if total target laps reached
            if laps_completed >= TARGET_LAPS:
                print(f"[FINISH] Target of {TARGET_LAPS} laps completed successfully!")
                cmd = CMD_STOP
                serial_ctrl.send_cmd(cmd)
                break

            serial_ctrl.send_cmd(cmd)

            # Display HUD telemetry
            hud_text = (f"Laps: {laps_completed}/{TARGET_LAPS} | LeftA: {left_area} | RightA: {right_area} | "
                        f"Angle: {angle:.1f} | Dir: {turn_dir} | CMD: {cmd}")
            cv2.putText(frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("ROBOVANGUARD - Round 1", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Manual quit requested.")
                serial_ctrl.send_cmd(CMD_STOP)
                break

            prev_diff = area_diff
            prev_angle = angle

    finally:
        cv2.destroyAllWindows()
        if HAS_PICAM2 and picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
        if cap is not None:
            cap.release()
        serial_ctrl.close()
        print("[INFO] Round 1 execution finished.")

if __name__ == "__main__":
    run_round1()