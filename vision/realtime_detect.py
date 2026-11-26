"""实时摄像头目标检测（WSL 端，GPU 推理）

运行:
    python realtime_detect.py --show

会自动在 Windows 侧拉起 camera_bridge.py（笔记本摄像头 -> MJPEG 流），
也可以通过 --source 指向已运行的推流地址或本地摄像头。

快捷键: q/ESC 退出, s 保存当前帧到 runs/realtime/
"""

import argparse
import json
import subprocess
import time
import urllib.request
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch
from ultralytics import YOLO

from yolo_demo import CN_NAMES, resolve_model

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "runs" / "realtime"
FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
]


def load_font(size=26):
    for path in FONT_CANDIDATES:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return None


def is_wsl():
    try:
        return "microsoft" in Path("/proc/version").read_text().lower()
    except OSError:
        return False


def bridge_status(url):
    base = url.rsplit("/", 1)[0]
    try:
        with urllib.request.urlopen(base + "/status", timeout=2) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return None


def ensure_bridge(url):
    if not url.startswith("http"):
        return
    if bridge_status(url):
        print("摄像头桥接服务已在运行")
        return
    if not is_wsl():
        print(f"桥接服务未运行，请先在 Windows 上执行: python {HERE / 'camera_bridge.py'}")
        return
    script = HERE / "camera_bridge.py"
    win_script = subprocess.run(["wslpath", "-w", str(script)],
                                capture_output=True, text=True).stdout.strip()
    win_cwd = subprocess.run(["wslpath", "-w", str(HERE)],
                             capture_output=True, text=True).stdout.strip()
    if not win_script:
        print("wslpath 不可用，请手动在 Windows 运行 camera_bridge.py")
        return
    print("正在 Windows 侧启动摄像头桥接服务 ...")
    subprocess.Popen(
        ["powershell.exe", "-NoProfile", "-Command",
         f"Start-Process python -WindowStyle Minimized -WorkingDirectory '{win_cwd}' "
         f"-ArgumentList '\"{win_script}\"'"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(40):
        time.sleep(0.5)
        if bridge_status(url):
            print("摄像头桥接服务已就绪")
            return
    print("桥接服务启动超时，请手动在 Windows 运行 camera_bridge.py")


def draw_panel(frame, lines, font):
    if font is None or not lines:
        return frame
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    pad, line_h = 12, font.size + 10
    width = int(max(probe.textlength(t, font=font) for t in lines)) + pad * 2
    height = line_h * len(lines) + pad
    panel = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(panel)
    draw.rounded_rectangle([0, 0, width - 1, height - 1], radius=10,
                           fill=(0, 0, 0, 160))
    for i, text in enumerate(lines):
        draw.text((pad, pad // 2 + i * line_h), text,
                  font=font, fill=(0, 255, 80, 255))
    rgba = np.array(panel)
    h, w = rgba.shape[:2]
    h = min(h, frame.shape[0] - 20)
    w = min(w, frame.shape[1] - 20)
    roi = frame[10:10 + h, 10:10 + w].astype(np.float32)
    alpha = (rgba[:h, :w, 3:4] / 255.0).astype(np.float32)
    blended = rgba[:h, :w, 2::-1] * alpha + roi * (1.0 - alpha)
    frame[10:10 + h, 10:10 + w] = blended.astype(np.uint8)
    return frame


def main():
    parser = argparse.ArgumentParser(description="实时摄像头 YOLO 检测")
    parser.add_argument("--source", default="http://localhost:8080/video",
                        help="MJPEG 地址，或本地摄像头编号 / 视频文件")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="", help="cuda:0 或 cpu，留空自动")
    parser.add_argument("--show", action="store_true", help="弹窗显示（WSLg）")
    parser.add_argument("--record", action="store_true", help="录制标注视频")
    parser.add_argument("--max-frames", type=int, default=0, help="0 表示不限")
    args = parser.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    if isinstance(source, str):
        ensure_bridge(source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(
            f"无法打开视频源: {args.source}\n"
            "1) Windows 端先运行: python camera_bridge.py\n"
            "2) 浏览器打开 http://localhost:8080/ 确认画面正常\n"
            "3) 仍失败可试 --source http://<Windows IP>:8080/video"
        )
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    model = YOLO(resolve_model(args.model))
    font = load_font()
    device = args.device or ("0" if torch.cuda.is_available() else "cpu")

    writer = None
    fps_ema, frame_count = 0.0, 0
    last_report = time.perf_counter()
    infer_ms = 0.0
    print(f"视频源: {args.source} | 模型: {args.model} | 设备: {device} | 按 q 退出")
    t_start = time.perf_counter()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取帧失败，视频源断开")
                break

            t0 = time.perf_counter()
            result = model.predict(frame, conf=args.conf, imgsz=args.imgsz,
                                   device=device, verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000

            annotated = result.plot()
            counts = Counter(result.names[int(c)] for c in result.boxes.cls.tolist())

            now = time.perf_counter()
            inst_fps = 1.0 / max(now - t0, 1e-6)
            fps_ema = inst_fps if fps_ema == 0 else fps_ema * 0.9 + inst_fps * 0.1

            summary = "  ".join(
                f"{CN_NAMES.get(n, n)} x{c}" for n, c in counts.most_common(4)
            ) or "无目标"
            panel = [
                f"FPS {fps_ema:4.1f}  推理 {infer_ms:4.1f}ms",
                f"目标 {sum(counts.values())}: {summary}",
            ]
            annotated = draw_panel(annotated, panel, font)

            frame_count += 1
            if args.record:
                if writer is None:
                    OUT_DIR.mkdir(parents=True, exist_ok=True)
                    h, w = annotated.shape[:2]
                    path = OUT_DIR / time.strftime("record_%Y%m%d_%H%M%S.mp4")
                    writer = cv2.VideoWriter(
                        str(path), cv2.VideoWriter_fourcc(*"mp4v"), 25, (w, h))
                    print(f"开始录制: {path}")
                writer.write(annotated)

            if args.show:
                cv2.imshow("YOLO realtime (q quit, s save)", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    OUT_DIR.mkdir(parents=True, exist_ok=True)
                    snap = OUT_DIR / time.strftime("snap_%Y%m%d_%H%M%S.jpg")
                    cv2.imwrite(str(snap), annotated)
                    print(f"已保存截图: {snap}")

            if now - last_report >= 2.0:
                print(f"[{frame_count:4d}帧] {fps_ema:5.1f} FPS | "
                      f"推理 {infer_ms:5.1f} ms | {summary}")
                last_report = now

            if args.max_frames and frame_count >= args.max_frames:
                break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
            print("录制结束")
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.perf_counter() - t_start
    print(f"共处理 {frame_count} 帧，{elapsed:.1f}s，平均 "
          f"{frame_count / max(elapsed, 1e-6):.1f} FPS")


if __name__ == "__main__":
    main()
