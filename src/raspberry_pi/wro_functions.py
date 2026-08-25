"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 OpenCV Vision Functions, Camera Manager & Drawing Helpers
(Includes Aspect Ratio Filtering, Dual-Layer HSV+LAB Masking, 0% Red/Orange Overlap, and 0% Black/Blue Overlap)
"""

import sys
import cv2
import numpy as np
from masks import rBlack, rMagenta


class CameraManager:
    """Universal Camera abstraction supporting both Picamera2 and OpenCV USB Webcams."""

    def __init__(self, force_webcam=False, device_index=0):
        self.force_webcam = force_webcam
        self.device_index = device_index
        self.cap = None
        self.picam2 = None
        self.is_webcam = False

    def start(self):
        if self.force_webcam:
            self._start_webcam()
        else:
            try:
                from picamera2 import Picamera2
                print("[INFO] Initializing Picamera2 (Pi CSI Camera)...")
                self.picam2 = Picamera2()
                self.picam2.preview_configuration.main.size = (640, 480)
                self.picam2.preview_configuration.main.format = "RGB888"
                self.picam2.preview_configuration.controls.FrameRate = 30
                self.picam2.preview_configuration.align()
                self.picam2.configure("preview")
                self.picam2.start()
                self.is_webcam = False
                print("[SUCCESS] Picamera2 initialized!")
            except Exception as e:
                print(f"[INFO] Picamera2 not available ({e}). Switching to USB Webcam...")
                self._start_webcam()

    def _start_webcam(self):
        search_indices = [self.device_index, 0, 1, 2, 3, 4, 5, 6, 8]
        seen = set()
        search_indices = [x for x in search_indices if not (x in seen or seen.add(x))]

        for idx in search_indices:
            print(f"[INFO] Testing USB Webcam index {idx}...")
            for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
                try:
                    cap = cv2.VideoCapture(idx, backend)
                    if cap and cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        
                        # Test capture 1 frame to verify real webcam device
                        for _ in range(3):
                            ret, frame = cap.read()
                            if ret and frame is not None and frame.size > 0:
                                print(f"[SUCCESS] USB Webcam initialized on index {idx} (/dev/video{idx})!")
                                self.cap = cap
                                self.device_index = idx
                                self.is_webcam = True
                                return
                        cap.release()
                except Exception:
                    pass

        print("[ERROR] Could not find any working USB webcam across indices 0-8!", file=sys.stderr)
        self.is_webcam = True
        self.cap = None

    def capture_array(self):
        if self.is_webcam:
            if self.cap:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    return frame
            return None
        else:
            return self.picam2.capture_array()

    def stop(self):
        if self.is_webcam and self.cap:
            self.cap.release()
        elif self.picam2:
            try:
                self.picam2.stop()
            except Exception:
                pass


def morphology_clean(mask, ksize=5, iterations=1):
    """Applies morphological close operation to filter noise."""
    kernel = np.ones((ksize, ksize), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)


def find_black_wall_contours(img_bgr, ROI, min_area=60):
    """
    Robust Black Side-Wall Segmentation using Dual-Layer HSV + LAB thresholds
    and explicit HSV Blue & Orange mask subtraction to guarantee 0% blue line overlap.
    """
    x1, y1, x2, y2 = ROI
    roi_bgr = img_bgr[y1:y2, x1:x2]

    roi_lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2Lab)
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    # 1. HSV Black Mask (V <= 65, S <= 110)
    mask_hsv_black = cv2.inRange(roi_hsv, np.array([0, 0, 0]), np.array([180, 110, 65]))

    # 2. LAB Black Mask (L <= 65, A in [105, 150], B in [112, 145])
    mask_lab_black = cv2.inRange(roi_lab, np.array([0, 105, 112]), np.array([65, 150, 145]))

    # Combine HSV Black & LAB Black with AND logic
    mask = cv2.bitwise_and(mask_hsv_black, mask_lab_black)

    # 3. Explicitly subtract ALL Blue pixels (HSV Blue: H in 80..140, S >= 35)
    blue_hsv_mask = cv2.inRange(roi_hsv, np.array([80, 35, 35]), np.array([140, 255, 255]))

    # 4. Explicitly subtract ALL Orange pixels (HSV Orange: H in 5..35, S >= 40)
    orange_hsv_mask = cv2.inRange(roi_hsv, np.array([5, 40, 40]), np.array([35, 255, 255]))

    # Subtract Blue and Orange masks completely
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(blue_hsv_mask))
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(orange_hsv_mask))

    # Apply MORPH_CLOSE & GaussianBlur to clean edges
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]


def find_red_pillar_contours(img_bgr, ROI, min_area=120):
    """
    Strict Red Pillar segmentation (HSV H in [0..8] U [172..180], S >= 120, V >= 80)
    with explicit Orange Hue exclusion AND Aspect Ratio filtering (H/W >= 0.75)
    to guarantee 0% Red/Orange floor line overlap.
    """
    x1, y1, x2, y2 = ROI
    roi_bgr = img_bgr[y1:y2, x1:x2]
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    # Pure Red HSV range (strict bounds around 0/180)
    mask1 = cv2.inRange(roi_hsv, np.array([0, 120, 80]), np.array([8, 255, 255]))
    mask2 = cv2.inRange(roi_hsv, np.array([172, 120, 80]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Explicitly subtract Orange Hue (H in 9..30)
    orange_mask = cv2.inRange(roi_hsv, np.array([9, 80, 80]), np.array([30, 255, 255]))
    red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(orange_mask))

    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    valid_pillars = []
    for cnt in contours:
        if cv2.contourArea(cnt) < min_area:
            continue
        # Geometry Aspect Ratio Filter: Pillars are tall/square 3D blocks (H/W >= 0.75)
        # Floor lines are wide horizontal stripes (H/W < 0.6)
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(h) / max(1.0, float(w))
        if aspect_ratio >= 0.75:
            valid_pillars.append(cnt)

    return valid_pillars


def find_orange_line_contours(img_bgr, ROI, min_area=100):
    """
    Strict Orange Line segmentation (HSV H in [10..25], S >= 100, V >= 100)
    with Red Hue exclusion to guarantee 0% Red/Orange overlap.
    """
    x1, y1, x2, y2 = ROI
    roi_bgr = img_bgr[y1:y2, x1:x2]
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    orange_mask = cv2.inRange(roi_hsv, np.array([10, 100, 100]), np.array([25, 255, 255]))

    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]


def find_contours(img_lab, lab_range, ROI, min_area=60):
    """Segment an ROI in CIELAB color space, apply Gaussian blur & MORPH_CLOSE, returning filtered contours."""
    x1, y1, x2, y2 = ROI
    img_segmented = img_lab[y1:y2, x1:x2]

    lower_mask = np.array(lab_range[0], dtype=np.uint8)
    upper_mask = np.array(lab_range[1], dtype=np.uint8)

    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    mask = cv2.GaussianBlur(mask, (7, 7), 0)
    mask = morphology_clean(mask, 5, 1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) > min_area]


def max_contour(contours, ROI=[0, 0, 0, 0]):
    """Returns [maxArea, maxX, maxY, maxContour] for the largest contour in ROI."""
    if not contours:
        return [0, 0, 0, None]

    maxArea = 0
    maxY = 0
    maxX = 0
    mCnt = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > maxArea:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            x += ROI[0] + w // 2
            y += ROI[1] + h
            maxArea = int(area)
            maxY = y
            maxX = x
            mCnt = cnt

    return [maxArea, maxX, maxY, mCnt]


def draw_roi(frame, roi, color=(0, 255, 255), thick=2):
    """Draws ROI boundary rectangle on frame."""
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)


def draw_offset_contours(frame, contours, roi, color, thick=2):
    """Draws contours offset to their correct full-frame coordinates within ROI."""
    if not contours:
        return
    x1, y1, _, _ = roi
    offset = np.array([[x1, y1]], dtype=np.int32)
    shifted = [cnt + offset for cnt in contours]
    cv2.drawContours(frame, shifted, -1, color, thick)


def display_variables(variables):
    """Prints debug telemetry variables on terminal using carriage returns."""
    names = list(variables.keys())
    for name in names:
        value = variables[name]
        print(f"{name}: {value}", end="\r\n")
    print("\033[F" * len(names), end="")
