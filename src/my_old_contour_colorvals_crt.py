# RPi4B — WRO Round 1 Vision → Serial commands ('f','l','r','s')
# Feed fix: retry camera init; fallback to QTGL preview; stable for autostart

import cv2
import numpy as np
import time
import serial
from picamera2 import Picamera2
from picamera2.previews import Preview  # import here to use later

# =========================
# TOGGLES
# =========================
USE_QTGL_PREVIEW = False  # set True if cv2.imshow stays black in your session

# =========================
# SERIAL / COMMANDS
# =========================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
SEND_INTERVAL_SEC = 0.05
APPEND_NEWLINE = True

CMD_RIGHT = 'r'
CMD_LEFT  = 'l'
CMD_FWD   = 'f'
CMD_STOP  = 's'

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
    time.sleep(2)  # allow ESP32 to reset
except Exception as e:
    print(f"[WARN] Serial not opened: {e}")

last_cmd = None
last_send_time = 0.0
def send_cmd(cmd):
    global last_cmd, last_send_time
    now = time.time()
    if (cmd != last_cmd) or (now - last_send_time >= SEND_INTERVAL_SEC):
        payload = (cmd + ("\n" if APPEND_NEWLINE else "")).encode()
        if ser:
            ser.write(payload)
        print(f"[TX] {cmd}")
        last_cmd = cmd
        last_send_time = now

# =========================
# CAMERA INIT WITH RETRY
# =========================
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
picam2 = None
for attempt in range(20):  # up to ~10 sec
    try:
        picam2 = Picamera2()
        picam2.preview_configuration.main.size = (FRAME_WIDTH, FRAME_HEIGHT)
        picam2.preview_configuration.main.format = "BGR888"
        picam2.preview_configuration.align()
        picam2.configure("preview")
        if USE_QTGL_PREVIEW:
            picam2.start_preview(Preview.QTGL)
        picam2.start()
        print(f"[INFO] Camera started on attempt {attempt+1}")
        break
    except Exception as e:
        print(f"[WARN] Camera not ready (attempt {attempt+1}): {e}")
        time.sleep(0.5)
else:
    raise RuntimeError("Camera __init__ sequence did not complete after retries.")

# =========================
# ROIs
# =========================
ROI1 = [20, 170, 240, 220]
ROI2 = [400, 170, 620, 220]
ROI3 = [200, 300, 440, 350]

# =========================
# CONSTANTS / GAINS
# =========================
kp = 0.02
kd = 0.006
straightConst = 87
turnThresh = 150
exitThresh = 1500
ANGLE_THRESH = 4.0
FAILSAFE_MIN_BLACK_AREA = 50

# Lab masks (tune to lighting)
lab_lower_black  = np.array([0,   0,   0],   np.uint8)
lab_upper_black  = np.array([70,  255, 255], np.uint8)
lab_lower_orange = np.array([40, 150, 150],  np.uint8)
lab_upper_orange = np.array([255,205, 255],  np.uint8)
lab_lower_blue   = np.array([20, 110,  0],   np.uint8)
lab_upper_blue   = np.array([255,170,110],   np.uint8)

MIN_AREA_LANE  = 60
MIN_AREA_INDIC = 80
INDIC_THRESH   = 100

# =========================
# HELPERS
# =========================
def morphology_clean(mask, ksize=5, iterations=1):
    kernel = np.ones((ksize, ksize), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)

def find_contours_lab(img_lab_roi, lower, upper, min_area):
    mask = cv2.inRange(img_lab_roi, lower, upper)
    mask = cv2.GaussianBlur(mask, (7, 7), 0)
    mask = morphology_clean(mask, 5, 1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]

def max_contour_area(contours):
    return int(max((cv2.contourArea(c) for c in contours), default=0))

def slice_roi(img, roi):
    x1, y1, x2, y2 = roi
    return img[y1:y2, x1:x2]

def draw_roi(frame, roi, color, thick=2):
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)

def draw_offset_contours(frame, contours, roi, color, thick=2):
    if not contours: return
    x1, y1, _, _ = roi
    offset = np.array([[x1, y1]], dtype=np.int32)
    shifted = [cnt + offset for cnt in contours]
    cv2.drawContours(frame, shifted, -1, color, thick)

# =========================
# STATE
# =========================
lTurn = False
rTurn = False
t = 0
turnDir = "none"
lDetected = False
prevDiff = 0.0
prevAngle = straightConst

if not USE_QTGL_PREVIEW:
    cv2.namedWindow("WRO R1 — Lab lanes + OB indicator → serial", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("WRO R1 — Lab lanes + OB indicator → serial", FRAME_WIDTH, FRAME_HEIGHT)

# =========================
# MAIN LOOP
# =========================
try:
    while True:
        frame_rgb = picam2.capture_array()
        if frame_rgb is None or frame_rgb.size == 0:
            print("[WARN] Empty frame; continuing…")
            continue
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        lab = cv2.GaussianBlur(lab, (7, 7), 0)

        roi_left_lab  = slice_roi(lab, ROI1)
        roi_right_lab = slice_roi(lab, ROI2)
        roi_ind_lab   = slice_roi(lab, ROI3)

        cLeft   = find_contours_lab(roi_left_lab,  lab_lower_black,  lab_upper_black,  MIN_AREA_LANE)
        cRight  = find_contours_lab(roi_right_lab, lab_lower_black,  lab_upper_black,  MIN_AREA_LANE)
        cOrange = find_contours_lab(roi_ind_lab,   lab_lower_orange, lab_upper_orange, MIN_AREA_INDIC)
        cBlue   = find_contours_lab(roi_ind_lab,   lab_lower_blue,   lab_upper_blue,   MIN_AREA_INDIC)

        leftArea  = max_contour_area(cLeft)
        rightArea = max_contour_area(cRight)
        orangeA   = max_contour_area(cOrange)
        blueA     = max_contour_area(cBlue)

        if orangeA > INDIC_THRESH:
            lDetected = True
            if turnDir == "none": turnDir = "right"
        elif blueA > INDIC_THRESH:
            lDetected = True
            if turnDir == "none": turnDir = "left"

        draw_roi(frame, ROI1, (0,255,255), 2)
        draw_roi(frame, ROI2, (0,255,255), 2)
        draw_roi(frame, ROI3, (255,255,0), 2)
        draw_offset_contours(frame, cLeft,  ROI1, (0,255,0), 2)
        draw_offset_contours(frame, cRight, ROI2, (0,255,0), 2)
        draw_offset_contours(frame, cOrange, ROI3, (0,165,255), 2)
        draw_offset_contours(frame, cBlue,   ROI3, (255,0,0), 2)

        aDiff = float(rightArea - leftArea)
        angle = straightConst + kp*aDiff + kd*(aDiff - prevDiff)

        if (leftArea <= turnThresh and not rTurn):
            lTurn = True
        elif (rightArea <= turnThresh and not lTurn):
            rTurn = True

        if (lTurn or rTurn) and ((rTurn and rightArea > exitThresh) or (lTurn and leftArea > exitThresh)):
            lTurn = rTurn = False
            prevDiff = 0.0
            if lDetected:
                t += 1
                lDetected = False
            turnDir = "none"

        if leftArea < FAILSAFE_MIN_BLACK_AREA and rightArea < FAILSAFE_MIN_BLACK_AREA:
            cmd = CMD_STOP
            lTurn = rTurn = False
        else:
            if lTurn:       cmd = CMD_LEFT
            elif rTurn:     cmd = CMD_RIGHT
            else:
                delta = angle - straightConst
                if   delta >=  ANGLE_THRESH: cmd = CMD_LEFT
                elif delta <= -ANGLE_THRESH: cmd = CMD_RIGHT
                else:                        cmd = CMD_FWD

        send_cmd(cmd)

        hud = (f"LeftA {leftArea:4d} | RightA {rightArea:4d} | Diff {aDiff:6.1f} | "
               f"Angle {angle:6.1f} | Turn L:{lTurn} R:{rTurn} | Indic O/B {orangeA}/{blueA} dir:{turnDir} | Laps {t} | CMD {cmd}")
        cv2.putText(frame, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (240,240,240), 2, cv2.LINE_AA)

        if USE_QTGL_PREVIEW:
            overlay = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            picam2.set_overlay(overlay)
        else:
            cv2.imshow("WRO R1 — Lab lanes + OB indicator → serial", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                send_cmd(CMD_STOP)
                break

        prevDiff = aDiff
        prevAngle = angle

finally:
    if not USE_QTGL_PREVIEW:
        cv2.destroyAllWindows()
    try:
        picam2.stop()
    except Exception:
        pass
    if ser:
        try:
            send_cmd(CMD_STOP)
            ser.close()
        except Exception:
            pass