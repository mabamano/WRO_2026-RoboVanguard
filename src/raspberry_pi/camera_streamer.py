"""
ROBOVANGUARD - WRO Future Engineers 2026
Raspberry Pi 5 Live MJPEG Camera Debug Streamer & Overlay

Provides:
1. Live web stream accessible via browser at http://<pi5_ip>:8080
2. Automatic frame saver ('latest_debug_frame.jpg')
3. Optional cv2.imshow GUI display fallback
"""

import time
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

latest_frame_jpeg = None
frame_lock = threading.Lock()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class CamStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Suppress HTTP access logging in terminal

    def do_GET(self):
        global latest_frame_jpeg
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>ROBOVANGUARD - Camera Debug Feed</title>
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <style>
                    body { background: #121212; color: #00ffcc; font-family: monospace; text-align: center; margin: 0; padding: 20px; }
                    h1 { margin-bottom: 10px; color: #ffffff; }
                    .container { display: inline-block; background: #1e1e1e; padding: 15px; border-radius: 10px; box-shadow: 0 0 20px rgba(0,255,204,0.3); }
                    img { max-width: 100%; height: auto; border: 2px solid #00ffcc; border-radius: 5px; }
                    .status { margin-top: 10px; font-size: 14px; color: #aaaaaa; }
                </style>
            </head>
            <body>
                <h1>🏎️ ROBOVANGUARD WRO Debug Stream</h1>
                <div class="container">
                    <img src="/stream.mjpg" alt="Live Camera Feed">
                    <div class="status">Raspberry Pi 5 Live Picamera2 Debug Stream (8080)</div>
                </div>
            </body>
            </html>
            """
            self.wfile.write(html.encode("utf-8"))
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with frame_lock:
                    if latest_frame_jpeg is None:
                        jpeg_bytes = None
                    else:
                        jpeg_bytes = latest_frame_jpeg

                if jpeg_bytes is not None:
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg_bytes)))
                        self.end_headers()
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b"\r\n")
                    except Exception:
                        break
                time.sleep(0.04)
        else:
            self.send_error(404)


class CameraDebugStreamer:
    """Manages background MJPEG web streaming & debug frame generation."""

    def __init__(self, port=8080):
        self.port = port
        self.server = None
        self.thread = None
        self.is_running = False

    def start(self):
        try:
            self.server = ThreadedHTTPServer(("0.0.0.0", self.port), CamStreamHandler)
            self.is_running = True
            self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.thread.start()
            print(f"[DEBUG STREAM] Web camera debug feed live at: http://localhost:{self.port} or http://<pi_ip>:{self.port}")
        except Exception as e:
            print(f"[WARNING] Could not start web debug server on port {self.port}: {e}")

    def update_frame(self, frame_bgr):
        global latest_frame_jpeg
        if frame_bgr is None:
            return

        success, encoded_img = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if success:
            jpeg_data = encoded_img.tobytes()
            with frame_lock:
                latest_frame_jpeg = jpeg_data

            # Save static frame image to disk for offline viewing
            try:
                cv2.imwrite("latest_debug_frame.jpg", frame_bgr)
            except Exception:
                pass

    def stop(self):
        self.is_running = False
        if self.server:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:
                pass
        print("[DEBUG STREAM] Web camera debug feed stopped.")
