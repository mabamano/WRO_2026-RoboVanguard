#!/usr/bin/env python3
"""
ROBOVANGUARD - USB Webcam Diagnostic Test Utility
Scans /dev/video0 through /dev/video8 to find and display connected USB Webcams.
"""

import sys
import time
import cv2

def main():
    print("=" * 65)
    print("   ROBOVANGUARD - USB Webcam Diagnostic Utility")
    print("=" * 65)

    working_caps = []
    print("[SCANNING] Checking video devices /dev/video0 through /dev/video8...")

    for idx in range(9):
        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            try:
                cap = cv2.VideoCapture(idx, backend)
                if cap and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        backend_name = "V4L2" if backend == cv2.CAP_V4L2 else "ANY"
                        print(f"[FOUND WEBCAM] Device index {idx} (/dev/video{idx}) working via {backend_name}!")
                        working_caps.append((idx, cap))
                        break
                    else:
                        cap.release()
            except Exception:
                pass

    if not working_caps:
        print("\n[ERROR] No working USB webcams found!")
        print("Troubleshooting steps:")
        print(" 1. Re-plug USB webcam into a blue USB 3.0 port on Pi 5.")
        print(" 2. Check if device is recognized in terminal: v4l2-ctl --list-devices or ls /dev/video*")
        return

    selected_idx, selected_cap = working_caps[0]
    print(f"\n[DISPLAY] Displaying live video stream from Webcam Index {selected_idx}...")
    print("Press 'q' or 'ESC' to exit.")

    window_name = f"USB Webcam Test (Device Index {selected_idx})"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    try:
        while True:
            ret, frame = selected_cap.read()
            if not ret or frame is None:
                print("[WARNING] Frame drop from webcam!")
                time.sleep(0.05)
                continue

            cv2.putText(frame, f"Webcam Index: {selected_idx} (640x480)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(window_name, frame)

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        for idx, cap in working_caps:
            cap.release()
        cv2.destroyAllWindows()
        print("[FINISHED] Webcam test closed.")

if __name__ == "__main__":
    main()
