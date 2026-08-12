#!/usr/bin/env python3
"""
ROBOVANGUARD - WRO Future Engineers
Round 2: Obstacle Avoidance & Parking Challenge Code

Description:
    Autonomous navigation algorithm for Round 2 (Obstacle Avoidance & Parking).
    Uses Raspberry Pi 5 + Pi Camera to detect obstacle sign boxes (Red box = turn right, Green box = turn left),
    track direction lines (Orange = clockwise, Blue = counter-clockwise), maintain lap counts, and trigger an
    automated parking routine into the parking lot (Magenta wall detection) after completing 3 laps.
    Sends serial UART commands ('f','l','r','s','p') to the ESP32 RoboGuard motor controller.

Team: ROBOVANGUARD (Ramco Institute of Technology)
"""

import sys
import time
import numpy as np
import cv2
import serial

HAS_PICAM2 = False
try:
    from picamera2 import Picamera2
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
CMD_PARK  = 'p'

FRAME_WIDTH, FRAME_HEIGHT = 640, 480
TARGET_LAPS = 3

# Regions of Interest [x1, y1, x2, y2]
ROI_OBSTACLE = [100, 150, 540, 380]  # Center zone for Red/Green box detection
ROI_LINE     = [200, 320, 440, 420]  # Orange/Blue direction indicator zone
ROI_PARK     = [150, 200, 490, 400]  # Magenta parking lot wall detection zone

# HSV Color Threshold Ranges for Obstacles & Track Features
# Red Obstacle Box (two hue ranges due to HSV wraparound)
HSV_LOWER_RED1 = np.array([0, 120, 70], np.uint8)
HSV_UPPER_RED1 = np.array([10, 255, 255], np.uint8)
HSV_LOWER_RED2 = np.array([170, 120, 70], np.uint8)
HSV_UPPER_RED2 = np.array([180, 255, 255], np.uint8)

# Green Obstacle Box
HSV_LOWER_GREEN = np.array([35, 80, 70], np.uint8)
HSV_UPPER_GREEN = np.array([85, 255, 255], np.uint8)

# Orange Line (Clockwise indicator)
HSV_LOWER_ORANGE = np.array([11, 150, 150], np.uint8)
HSV_UPPER_ORANGE = np.array([25, 255, 255], np.uint8)

# Blue Line (Counter-clockwise indicator)
HSV_LOWER_BLUE = np.array([95, 120, 100], np.uint8)
HSV_UPPER_BLUE = np.array([130, 255, 255], np.uint8)

# Magenta Zone (Parking wall/lot indicator)
HSV_LOWER_MAGENTA = np.array([140, 100, 100], np.uint8)
HSV_UPPER_MAGENTA = np.array([165, 255, 255], np.uint8)

MIN_AREA_OBSTACLE = 800
MIN_AREA_LINE     = 150
MIN_AREA_PARK     = 1200

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
            time.sleep(2.0)
            print(f"[INFO] Serial interface opened on {port} @ {baud} baud.")
        except Exception as err:
            print(f"[WARN] Serial port could not be opened: {err}")

    def send_cmd(self, cmd):
        """Send movement command character to ESP32."""
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
        """Send stop command and close serial port."""
        if self.ser and self.ser.is_open:
            try:
                self.send_cmd(CMD_STOP)
                self.ser.close()
            except Exception:
                pass

# ==============================================================================
# VISION PROCESSING HELPERS
# ==============================================================================
def find_contours_hsv(img_hsv_roi, lower, upper, min_area, lower2=None, upper2=None):
    """Filters image in HSV color space (with optional second hue band) and returns contours."""
    mask = cv2.inRange(img_hsv_roi, lower, upper)
    if lower2 is not None and upper2 is not None:
        mask2 = cv2.inRange(img_hsv_roi, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]

def max_contour_area(contours):
    """Returns max area of a contour list."""
    return int(max((cv2.contourArea(c) for c in contours), default=0))

def slice_roi(img, roi):
    """Slices image array according to ROI bounding box."""
    x1, y1, x2, y2 = roi
    return img[y1:y2, x1:x2]

def draw_roi(frame, roi, color, thick=2):
    """Draws ROI rectangle on visualization overlay."""
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)

def draw_offset_contours(frame, contours, roi, color, thick=2):
    """Draws contours offset back to main image frame coordinates."""
    if not contours:
        return
    x1, y1, _, _ = roi
    offset = np.array([[x1, y1]], dtype=np.int32)
    shifted = [cnt + offset for cnt in contours]
    cv2.drawContours(frame, shifted, -1, color, thick)

# ==============================================================================
# MAIN ROUTINE FOR ROUND 2
# ==============================================================================
def run_round2():
    print("=" * 60)
    print("🤖 ROBOVANGUARD - Starting Round 2 (Obstacle Avoidance & Parking)")
    print("=" * 60)

    serial_ctrl = SerialController()

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
                print(f"[INFO] Picamera2 started successfully on attempt {attempt+1}")
                break
            except Exception as e:
                print(f"[WARN] Camera init retry ({attempt+1}): {e}")
                time.sleep(0.5)
        else:
            HAS_PICAM2 = False

    if not HAS_PICAM2:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # State tracking
    laps_completed = 0
    in_parking_phase = False
    direction_mode = "unknown"
    last_line_time = 0.0

    cv2.namedWindow("ROBOVANGUARD - Round 2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ROBOVANGUARD - Round 2", FRAME_WIDTH, FRAME_HEIGHT)

    try:
        while True:
            if HAS_PICAM2 and picam2 is not None:
                frame_rgb = picam2.capture_array()
                if frame_rgb is None or frame_rgb.size == 0:
                    continue
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.03)
                    continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Extract ROIs
            roi_obs_hsv  = slice_roi(hsv, ROI_OBSTACLE)
            roi_line_hsv = slice_roi(hsv, ROI_LINE)
            roi_park_hsv = slice_roi(hsv, ROI_PARK)

            # Detect obstacle boxes (Red / Green)
            c_red   = find_contours_hsv(roi_obs_hsv, HSV_LOWER_RED1, HSV_UPPER_RED1, MIN_AREA_OBSTACLE, HSV_LOWER_RED2, HSV_UPPER_RED2)
            c_green = find_contours_hsv(roi_obs_hsv, HSV_LOWER_GREEN, HSV_UPPER_GREEN, MIN_AREA_OBSTACLE)

            # Detect direction lines (Orange / Blue)
            c_orange = find_contours_hsv(roi_line_hsv, HSV_LOWER_ORANGE, HSV_UPPER_ORANGE, MIN_AREA_LINE)
            c_blue   = find_contours_hsv(roi_line_hsv, HSV_LOWER_BLUE, HSV_UPPER_BLUE, MIN_AREA_LINE)

            # Detect parking magenta zone
            c_magenta = find_contours_hsv(roi_park_hsv, HSV_LOWER_MAGENTA, HSV_UPPER_MAGENTA, MIN_AREA_PARK)

            red_area     = max_contour_area(c_red)
            green_area   = max_contour_area(c_green)
            orange_area  = max_contour_area(c_orange)
            blue_area    = max_contour_area(c_blue)
            magenta_area = max_contour_area(c_magenta)

            # Check track orientation line
            now = time.time()
            if (orange_area > MIN_AREA_LINE or blue_area > MIN_AREA_LINE) and (now - last_line_time > 3.0):
                if orange_area > blue_area:
                    direction_mode = "Clockwise (Orange Line)"
                else:
                    direction_mode = "Counter-Clockwise (Blue Line)"
                laps_completed += 1
                last_line_time = now
                print(f"[TRACK] Lap {laps_completed}/{TARGET_LAPS} detected! Mode: {direction_mode}")

            # Draw visual HUD overlays
            draw_roi(frame, ROI_OBSTACLE, (255, 255, 255), 2)
            draw_roi(frame, ROI_LINE,     (255, 255, 0), 2)
            draw_roi(frame, ROI_PARK,     (255, 0, 255), 2)

            draw_offset_contours(frame, c_red,     ROI_OBSTACLE, (0, 0, 255), 2)
            draw_offset_contours(frame, c_green,   ROI_OBSTACLE, (0, 255, 0), 2)
            draw_offset_contours(frame, c_orange,  ROI_LINE,     (0, 165, 255), 2)
            draw_offset_contours(frame, c_blue,    ROI_LINE,     (255, 0, 0), 2)
            draw_offset_contours(frame, c_magenta, ROI_PARK,     (255, 0, 255), 2)

            # Check parking sequence trigger after lap 3 completion
            if laps_completed >= TARGET_LAPS and magenta_area > MIN_AREA_PARK:
                in_parking_phase = True
                print("[PARKING] Magenta zone detected! Triggering parking sequence.")
                cmd = CMD_PARK
                serial_ctrl.send_cmd(cmd)
                time.sleep(2.0)
                serial_ctrl.send_cmd(CMD_STOP)
                break

            # Obstacle avoidance steering arbitration
            # Red box = turn RIGHT, Green box = turn LEFT
            if red_area > green_area and red_area > MIN_AREA_OBSTACLE:
                cmd = CMD_RIGHT
                action_text = "OBSTACLE: RED BOX -> TURN RIGHT"
            elif green_area > red_area and green_area > MIN_AREA_OBSTACLE:
                cmd = CMD_LEFT
                action_text = "OBSTACLE: GREEN BOX -> TURN LEFT"
            else:
                cmd = CMD_FWD
                action_text = "PATH CLEAR -> FORWARD"

            serial_ctrl.send_cmd(cmd)

            # HUD Display
            hud_info = f"Lap: {laps_completed}/{TARGET_LAPS} | {action_text} | CMD: {cmd}"
            cv2.putText(frame, hud_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("ROBOVANGUARD - Round 2", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Manual exit.")
                serial_ctrl.send_cmd(CMD_STOP)
                break

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
        print("[INFO] Round 2 execution finished.")

if __name__ == "__main__":
    run_round2()
