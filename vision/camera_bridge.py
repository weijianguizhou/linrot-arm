"""笔记本摄像头 MJPEG 推流服务（在 Windows 上运行）

用法（Windows PowerShell / cmd，不是 WSL）:
    python camera_bridge.py                 # 默认 8080 端口，1280x720
    python camera_bridge.py --port 8090 --width 960 --height 540

浏览器预览: http://localhost:8080/
WSL 里作为 YOLO 输入: --source http://<Windows主机IP>:8080/video
"""

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = b"frame"


class Camera:
    def __init__(self, index, width, height, fps, quality):
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise SystemExit(f"无法打开摄像头 {index}，检查是否被其他程序占用")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.fps = fps
        self.quality = quality
        self.jpeg = None
        self.size = (0, 0)
        self.real_fps = 0.0
        self.clients = 0
        self.last_seen = time.time()
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        frames, t0 = 0, time.time()
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            )
            if not ok:
                continue
            with self.lock:
                self.jpeg = buf.tobytes()
                self.size = (frame.shape[1], frame.shape[0])
            frames += 1
            now = time.time()
            if now - t0 >= 1.0:
                self.real_fps = frames / (now - t0)
                frames, t0 = 0, now

    def latest(self):
        with self.lock:
            return self.jpeg


def make_handler(camera):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                body = (
                    "<html><body style='margin:0;background:#111'>"
                    "<img src='/video' style='width:100vw'>"
                    "</body></html>"
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/snapshot":
                jpeg = camera.latest()
                if jpeg is None:
                    self.send_error(503, "no frame")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            elif path == "/video":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode(),
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                camera.clients += 1
                try:
                    while True:
                        camera.last_seen = time.time()
                        jpeg = camera.latest()
                        if jpeg is not None:
                            self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                            )
                            self.wfile.write(jpeg + b"\r\n")
                        time.sleep(1.0 / max(camera.fps, 1))
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    camera.clients -= 1
            elif path == "/status":
                body = json.dumps(
                    {
                        "size": camera.size,
                        "camera_fps": round(camera.real_fps, 1),
                        "quality": camera.quality,
                    }
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

    return Handler


def main():
    parser = argparse.ArgumentParser(description="笔记本摄像头 MJPEG 推流服务")
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--quality", type=int, default=80, help="JPEG 质量 1-100")
    parser.add_argument("--idle-timeout", type=int, default=120,
                        help="无客户端连接多少秒后自动退出，0 表示不退出")
    args = parser.parse_args()

    camera = Camera(args.camera, args.width, args.height, args.fps, args.quality)
    time.sleep(1.0)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(camera))

    if args.idle_timeout > 0:
        def watchdog():
            while True:
                time.sleep(5)
                idle = time.time() - camera.last_seen
                if camera.clients == 0 and idle > args.idle_timeout:
                    print(f"空闲 {idle:.0f}s，自动退出")
                    os._exit(0)

        threading.Thread(target=watchdog, daemon=True).start()

    print(f"摄像头推流已启动: http://0.0.0.0:{args.port}/video")
    print("Ctrl+C 退出")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        camera.running = False
        camera.cap.release()
        server.server_close()


if __name__ == "__main__":
    main()
