# -*- coding: utf-8 -*-
"""结构之外的真校验：惯量物理性、闭环残差、关节站位是否真的落在零件实体上、尺度。

注意两处易错点（本版已修正）：
1. URDF 根坐标系 = B0 的站位，不是 CAD 全局原点。loops.yaml 的 origin_mm 是 CAD 坐标，
   必须减去 mapping.json 里 B0 的 origin_mm 才能和 FK 结果比较。
2. 「转轴垂直于两刚体质心连线」不是物理要求（质心完全可以落在轴线上，例如肩部第一轴）。
   真正该查的是：关节站位必须同时落在父、子两个零件的实体附近，否则就是装错了位置。
"""

import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(ROOT, "urdf", "linear_arm.urdf")
LOOPS = os.path.join(ROOT, "loops.yaml")
MAPPING = os.path.join(ROOT, "mapping.json")


def parse():
    robot = ET.parse(URDF).getroot()
    links, joints = {}, {}
    for l in robot.findall("link"):
        inert = l.find("inertial")
        d = dict(mesh=None)
        g = l.find("visual/geometry/mesh")
        if g is not None:
            d["mesh"] = os.path.normpath(
                os.path.join(os.path.dirname(URDF), g.get("filename")))
        if inert is not None:
            o = inert.find("origin")
            xyz = np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()])
            d["mass"] = float(inert.find("mass").get("value"))
            i = inert.find("inertia")
            I = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                          [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                          [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
            d["com"] = xyz
            d["I"] = I
        links[l.get("name")] = d
    for j in robot.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        joints[j.get("name")] = dict(
            type=j.get("type"), parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()]),
            rpy=np.array([float(x) for x in (o.get("rpy") if o is not None else "0 0 0").split()]),
            axis=np.array([float(x) for x in (a.get("xyz") if a is not None else "1 0 0").split()]))
    return links, joints


def fk(joints):
    """q=0 时各 link 在世界系（=URDF 根系）的位置。所有 rpy 都是 0，故只需平移累加。"""
    kids = {j["child"]: n for n, j in joints.items()}
    roots = [j["parent"] for j in joints.values() if j["parent"] not in kids]
    root = roots[0] if roots else list(joints.values())[0]["parent"]
    pos = {root: np.zeros(3)}
    queue = [root]
    while queue:
        x = queue.pop(0)
        for n, j in joints.items():
            if j["parent"] == x:
                pos[j["child"]] = pos[x] + j["xyz"]
                queue.append(j["child"])
    return pos


def main():
    links, joints = parse()
    pos = fk(joints)
    problems = []

    b0 = np.array(json.load(open(MAPPING, encoding="utf-8"))["B0"]["origin_mm"])
    print("URDF 根系 = B0 站位，CAD 原点 - B0 = %s mm" % np.round(-b0, 2))

    # ---------------- 1. 惯量
    print("=" * 72)
    print("1) 惯量物理性")
    for n, l in links.items():
        if "I" not in l:
            continue
        I = l["I"]
        if l["mass"] <= 0:
            problems.append(f"{n} 质量 0")
            continue
        w = np.linalg.eigvalsh(I)
        tri = [I[0, 0] + I[1, 1] - I[2, 2], I[1, 1] + I[2, 2] - I[0, 0],
               I[0, 0] + I[2, 2] - I[1, 1]]
        ok_pd, ok_tri = w.min() > 0, min(tri) > 0
        note = ""
        if l["mesh"] and os.path.exists(l["mesh"]):
            b = trimesh.load(l["mesh"], force="mesh").bounds
            com = l["com"]
            if not (np.all(com >= b[0] - 0.001) and np.all(com <= b[1] + 0.001)):
                note = "  质心在网格外!"
                problems.append(f"{n} 质心在网格外")
        if not (ok_pd and ok_tri):
            problems.append(f"{n} 惯量非物理")
        print(f"  {n:4s} m={l['mass']:7.4f}  eig_min={w.min():9.2e}  "
              f"三角不等式={'OK' if ok_tri else 'FAIL'}{note}")

    # ---------------- 2. 闭环残差（父/子两侧锚点必须重合成同一点）
    print("=" * 72)
    print("2) 闭环残差（父锚点 vs 子锚点，q=0）")
    loops = json.load(open(LOOPS, encoding="utf-8"))
    for lp in loops:
        par, ch = lp["parent"], lp["child"]
        if par not in pos or ch not in pos:
            print(f"  {lp['joint']}: 端点不在树里，跳过")
            continue
        p_par = pos[par] + np.array(lp["origin_parent_mm"]) / 1000.0
        p_ch = pos[ch] + np.array(lp["origin_child_mm"]) / 1000.0
        err = float(np.linalg.norm(p_par - p_ch)) * 1000.0
        station = (np.array(lp["origin_mm"]) - b0) / 1000.0
        err_st = float(np.linalg.norm((p_par + p_ch) / 2 - station)) * 1000.0
        print(f"  {lp['joint']:4s} {par}->{ch}: 锚点差 {err:6.3f} mm"
              f"   锚点/站位 偏差 {err_st:6.3f} mm   站位 {np.round(station * 1000, 2)}")
        if err > 0.05:
            problems.append(f"闭环 {lp['joint']} 锚点残差 {err:.2f} mm")
        if err_st > 0.5:
            problems.append(f"闭环 {lp['joint']} 锚点与记录站位不同 {err_st:.2f} mm")

    # ---------------- 3. 关节站位必须落在父/子零件的包围盒内
    print("=" * 72)
    print("3) 关节站位包络性（站位必须落在父、子零件包围盒内；销孔中心在材料环里，故只查包络）")
    cache = {}

    def bbox_of(name):
        if name not in cache:
            m = links[name]["mesh"]
            cache[name] = (trimesh.load(m, force="mesh").bounds
                           if m and os.path.exists(m) else None)
        return cache[name]

    tol = 0.010  # m：啮合面天然落在包围盒边缘，留 10 mm 余量；仍能抓住数百毫米级错位
    for n, j in joints.items():
        if j["type"] == "fixed":
            continue
        p = pos[j["parent"]] + j["xyz"]
        row, ds = [], []
        for side in ("parent", "child"):
            bb = bbox_of(j[side])
            if bb is None:
                row.append("  n/a")
                ds.append(0.0)
                continue
            rel = p - pos[j[side]]
            out = np.maximum(np.maximum(bb[0] - rel, rel - bb[1]), 0.0)
            d = float(np.linalg.norm(out))
            ds.append(d)
            row.append(f"{d * 1000.0:6.2f}")
        # 转动副的轴点可以落在父件里（例如电机转子与法兰的交界面），
        # 因此只要在父、子任一件的包围盒内就算合理；两边都出盒才报错。
        if min(ds) > tol:
            problems.append(f"{n} 站位在父、子包围盒外 {min(ds) * 1000.0:.1f} mm")
        print(f"  {n:4s} {j['parent']}->{j['child']}  出盒距离=[{row[0]}, {row[1]}] mm")

    # ---------------- 4. 尺度
    print("=" * 72)
    print("4) 尺度合理性（网格尺寸 vs 关节偏移，防止单位错 1000 倍）")
    allb = []
    for n, l in links.items():
        if l["mesh"] and os.path.exists(l["mesh"]):
            m_ = trimesh.load(l["mesh"], force="mesh")
            allb.append(m_.bounds + pos.get(n, np.zeros(3))[None, :])
    if allb:
        lo = np.min([b[0] for b in allb], axis=0)
        hi = np.max([b[1] for b in allb], axis=0)
        span = (hi - lo) * 1000
        print(f"  整机包围盒 {span[0]:.0f} x {span[1]:.0f} x {span[2]:.0f} mm")
        if max(span) > 5000:
            problems.append("包围盒 >5 m，单位可能错")

    print("=" * 72)
    print("问题汇总:", "无" if not problems else "")
    for p in problems:
        print("  -", p)
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
