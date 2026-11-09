"""重建 stl_map.json：用**完整文件名**精确匹配，不再用前 12 字符前缀。

之前用前缀匹配，`直线模组手臂part56_腕部主支架` / `腕部十字轴` / `腕部连接件`
全都撞在 `直线模组手臂part56_腕` 上，导致 7 个刚体配错网格。
"""

import json
import os
import re

HERE = r"D:\robotics\_research"
STL_DIR = os.path.join(HERE, "stl")
MODEL_DIR = r"D:\robotics\linear_arm_model\linear_arm_model"
OUT = os.path.join(HERE, "stl_map.json")

world = json.load(open(os.path.join(HERE, "world.json"), encoding="utf-8"))
files = os.listdir(STL_DIR)
on_disk = {f.lower(): f for f in os.listdir(MODEL_DIR)}


def strip_idx(name):
    """'12_直线模组手臂腕部2.STL' -> '直线模组手臂腕部2'"""
    m = re.match(r"^[X]?\d+_(.*)$", name)
    return (m.group(1) if m else name).rsplit(".", 1)[0].lower()


by_stem = {}
for f in files:
    by_stem.setdefault(strip_idx(f), []).append(f)

mapping, miss = {}, []
for key, v in world.items():
    path = v.get("path") or ""
    if not path.lower().endswith(".sldprt"):
        continue
    stem = os.path.splitext(os.path.basename(path))[0].lower()
    hit = by_stem.get(stem)
    if hit:
        mapping[key] = os.path.join(STL_DIR, hit[0])
        continue
    # 虚拟零件：装配里记的路径带重复前缀，用磁盘上的真实文件名兜底
    cand = None
    for low, real in on_disk.items():
        rs = os.path.splitext(real)[0].lower()
        if stem.endswith(rs) or rs.endswith(stem[-14:]):
            cand = rs
            break
    if cand and by_stem.get(cand):
        mapping[key] = os.path.join(STL_DIR, by_stem[cand][0])
    else:
        miss.append(key)

json.dump(mapping, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print("mapped", len(mapping), "missing", len(miss))
for m in miss:
    print("  MISS", m.split("/")[-1][:50])
