"""YOLO 目标检测 Demo

用法:
    python yolo_demo.py                       # 用内置示例图片
    python yolo_demo.py --show                # 弹窗显示检测结果
    python yolo_demo.py --source photo.jpg    # 检测自己的图片
    python yolo_demo.py --source video.mp4 --show
    python yolo_demo.py --source 0 --show     # 摄像头（WSL 需 usbipd 直通）
    python yolo_demo.py --model yolo11s.pt    # 换更大的模型
"""

import argparse
import subprocess
import time
from collections import Counter
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
SAMPLE_URLS = [
    "https://ultralytics.com/images/bus.jpg",
]
SAMPLE_PATH = HERE / "data" / "bus.jpg"
WEIGHTS_DIR = HERE / "weights"
WEIGHT_MIRRORS = [
    "https://hf-mirror.com/Ultralytics/YOLO11/resolve/main/{name}",
    "https://hf-mirror.com/Ultralytics/YOLOv8/resolve/main/{name}",
]
VIDEO_SUFFIXES = {".mp4", ".avi", ".mov", ".mkv", ".webm"}

CN_NAMES = {
    "person": "人", "bicycle": "自行车", "car": "汽车", "motorcycle": "摩托车",
    "airplane": "飞机", "bus": "公交车", "train": "火车", "truck": "卡车",
    "boat": "船", "traffic light": "交通灯", "fire hydrant": "消防栓",
    "stop sign": "停止标志", "parking meter": "停车计时器", "bench": "长椅",
    "bird": "鸟", "cat": "猫", "dog": "狗", "horse": "马", "sheep": "羊",
    "cow": "牛", "elephant": "大象", "bear": "熊", "zebra": "斑马",
    "giraffe": "长颈鹿", "backpack": "背包", "umbrella": "雨伞",
    "handbag": "手提包", "tie": "领带", "suitcase": "行李箱", "frisbee": "飞盘",
    "skis": "滑雪板", "snowboard": "单板滑雪板", "sports ball": "球",
    "kite": "风筝", "baseball bat": "棒球棒", "baseball glove": "棒球手套",
    "skateboard": "滑板", "surfboard": "冲浪板", "tennis racket": "网球拍",
    "bottle": "瓶子", "wine glass": "酒杯", "cup": "杯子", "fork": "叉子",
    "knife": "刀", "spoon": "勺子", "bowl": "碗", "banana": "香蕉",
    "apple": "苹果", "sandwich": "三明治", "orange": "橙子",
    "broccoli": "西兰花", "carrot": "胡萝卜", "hot dog": "热狗", "pizza": "披萨",
    "donut": "甜甜圈", "cake": "蛋糕", "chair": "椅子", "couch": "沙发",
    "potted plant": "盆栽", "bed": "床", "dining table": "餐桌", "toilet": "马桶",
    "tv": "电视", "laptop": "笔记本电脑", "mouse": "鼠标", "remote": "遥控器",
    "keyboard": "键盘", "cell phone": "手机", "microwave": "微波炉",
    "oven": "烤箱", "toaster": "烤面包机", "sink": "水槽", "refrigerator": "冰箱",
    "book": "书", "clock": "时钟", "vase": "花瓶", "scissors": "剪刀",
    "teddy bear": "泰迪熊", "hair drier": "吹风机", "toothbrush": "牙刷",
}


def download(url, dst, retries=3):
    dst = Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(dst.suffix + ".part")
    try:
        subprocess.run(
            ["curl", "-fsSL", "--retry", str(retries), "--connect-timeout", "15",
             "--max-time", "600", "-o", str(tmp), url],
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        from urllib.request import urlretrieve
        urlretrieve(url, tmp)
    tmp.replace(dst)
    return dst


def ensure_sample():
    if SAMPLE_PATH.exists():
        return str(SAMPLE_PATH)
    for url in SAMPLE_URLS:
        try:
            print(f"下载示例图片: {url}")
            return str(download(url, SAMPLE_PATH))
        except Exception as exc:
            print(f"  失败: {exc}")
    raise SystemExit("示例图片下载失败，请用 --source 指定本地图片/视频")


def resolve_model(name):
    path = Path(name).expanduser()
    if path.exists() or path.suffix != ".pt":
        return name
    local = WEIGHTS_DIR / path.name
    if local.exists():
        return str(local)
    for pattern in WEIGHT_MIRRORS:
        url = pattern.format(name=path.name)
        try:
            print(f"下载权重: {url}")
            return str(download(url, local))
        except Exception as exc:
            print(f"  失败: {exc}")
    print("镜像下载失败，尝试交给 ultralytics 官方源（需要访问 GitHub）")
    return name


def parse_source(raw):
    if raw == "":
        return ensure_sample(), False
    if raw.isdigit():
        return int(raw), True
    path = Path(raw).expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"找不到源文件: {path}")
    return str(path), path.suffix.lower() in VIDEO_SUFFIXES


def main():
    parser = argparse.ArgumentParser(description="YOLO 目标检测 Demo")
    parser.add_argument("--source", default="",
                        help="图片/视频路径或摄像头编号，留空使用示例图片")
    parser.add_argument("--model", default="yolo11n.pt",
                        help="模型权重，默认 yolo11n.pt（自动下载）")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--device", default="", help="cuda:0 或 cpu，留空自动")
    parser.add_argument("--show", action="store_true", help="弹窗实时显示结果")
    args = parser.parse_args()

    source, streaming = parse_source(args.source)

    weights = resolve_model(args.model)
    model = YOLO(weights)
    device = args.device or None

    print(f"模型: {weights} | 设备: {device or '自动'} | 源: {source}")
    start = time.perf_counter()
    results = model.predict(
        source=source, conf=args.conf, device=device, stream=streaming,
        show=args.show, save=True, project=str(HERE / "runs"), name="demo",
        exist_ok=True, verbose=False,
    )

    counts, confs, frames, elapsed = Counter(), {}, 0, 0.0
    last = None
    for result in results:
        if frames:
            elapsed += result.speed.get("inference", 0.0)
        frames += 1
        last = result
        for cls_id, conf in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist()):
            name = result.names[int(cls_id)]
            counts[name] += 1
            confs.setdefault(name, []).append(conf)

    total = time.perf_counter() - start
    print(f"\n推理完成: {frames} 帧, 总耗时 {total:.2f}s")
    if not counts:
        print("未检测到任何目标，可尝试调低 --conf")
        return

    print(f"共检测到 {sum(counts.values())} 个目标:")
    for name, n in counts.most_common():
        avg = sum(confs[name]) / n
        best = max(confs[name])
        print(f"  {CN_NAMES.get(name, name):<8}({name:<14}) x{n:<3} "
              f"平均置信度 {avg:.2f} / 最高 {best:.2f}")

    if last is not None:
        saved = Path(last.save_dir)
        print(f"\n标注结果已保存到: {saved}")
        for f in sorted(saved.iterdir()):
            if f.is_file():
                print(f"  {f}")


if __name__ == "__main__":
    main()
