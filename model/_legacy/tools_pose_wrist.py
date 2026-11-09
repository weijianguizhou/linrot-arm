# -*- coding: utf-8 -*-
"""腕部姿态手术：把 {B7,B8,B9,B10,B11} 绕 R7/R8 的公共轴线刚性转 theta。

原理：R7、R8 轴都是 Y 向，站位 (-0.083, *, -0.5064) —— 两点共线，所以这一段
刚体（模组+活塞杆+端板）可以整体绕这条轴转，环自然保持闭合。
落地方式：只改 R7、R8 两个 joint 的 origin（加 rpy），其余网格不动。

用法：python tools_pose_wrist.py --theta -90          # 写回 urdf
      python tools_pose_wrist.py --theta -90 --dry    # 只报告，不写
"""
from __future__ import annotations

import argparse
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

V2 = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2"
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")


def rpy_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_rpy(R):
    """旋转矩阵 -> URDF 的 rpy（固定轴 X-Y-Z，等价于 Rz@Ry@Rx）。"""
    sy = -R[2, 0]
    sy = float(np.clip(sy, -1.0, 1.0))
    pitch = np.arcsin(sy)
    if abs(abs(sy) - 1.0) < 1e-9:
        roll = 0.0
        yaw = np.arctan2(R[0, 1], R[0, 2])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def axis_angle(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    th = np.deg2rad(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def parse(path):
    t = ET.parse(path)
    root = t.getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        joints[j.get("name")] = dict(
            el=j,
            xyz=np.array([float(x) for x in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3),
            rpy=np.array([float(x) for x in o.get("rpy", "0 0 0").split()]) if o is not None else np.zeros(3),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            axis=np.array([float(x) for x in j.find("axis").get("xyz", "1 0 0").split()])
            if j.find("axis") is not None else np.array([1.0, 0, 0]),
        )
    return t, root, joints


def fk(joints):
    links = set()
    for j in joints.values():
        links.add(j["parent"])
        links.add(j["child"])
    kids = {j["child"] for j in joints.values()}
    root = [l for l in links if l not in kids][0]
    T = {root: np.eye(4)}
    changed = True
    while changed:
        changed = False
        for j in joints.values():
            if j["parent"] in T and j["child"] not in T:
                M = np.eye(4)
                M[:3, :3] = rpy_matrix(*j["rpy"])
                M[:3, 3] = j["xyz"]
                T[j["child"]] = T[j["parent"]] @ M
                changed = True
    return root, T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--theta", type=float, default=0.0)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    t, root_el, joints = parse(URDF)
    root, T = fk(joints)

    # 检查 R7 / R8 是否共轴（都沿 Y、站位的 x,z 相同）
    r7, r8 = joints["R7"], joints["R8"]
    p7 = (T[r7["parent"]] @ np.r_[r7["xyz"], 1])[:3]
    p8 = (T[r8["parent"]] @ np.r_[r8["xyz"], 1])[:3]
    a7 = T[r7["parent"]][:3, :3] @ r7["axis"]
    a8 = T[r8["parent"]][:3, :3] @ r8["axis"]
    print(f"R7 轴点 {np.round(p7*1000,2)} mm 轴 {np.round(a7,3)}")
    print(f"R8 轴点 {np.round(p8*1000,2)} mm 轴 {np.round(a8,3)}")
    d_ax = np.linalg.norm(np.cross(p8 - p7, a7))
    print(f"两轴平行度 {np.degrees(np.arccos(min(1,abs(a7@a8)))):.3f}°，异面距离 {d_ax*1000:.2f} mm")

    if abs(args.theta) > 1e-9:
        R = axis_angle(a7, args.theta)
        p_axis = p7
        for jn in ("R7", "R8"):
            j = joints[jn]
            Tp = T[j["parent"]]
            # 原 origin（父->子，纯平移）
            t_old = j["xyz"]
            t_new = R @ t_old + (np.eye(3) - R) @ (np.linalg.inv(Tp[:3, :3]) @ (p_axis - Tp[:3, 3]))
            rpy_new = matrix_rpy(np.linalg.inv(Tp[:3, :3]) @ R @ Tp[:3, :3])
            j["xyz"], j["rpy"] = t_new, np.array(rpy_new)
            # 轴向不用动：关节仍绕同一条物理轴转，只是零位变了
        print(f"\n已把 R7/R8 绕公共轴转 {args.theta}°")

    # 写回
    for jn, j in joints.items():
        el = j["el"]
        o = el.find("origin")
        o.set("xyz", " ".join(f"{x:.6g}" for x in j["xyz"]))
        o.set("rpy", " ".join(f"{x:.6g}" for x in j["rpy"]))
        ax = el.find("axis")
        if ax is not None:
            a = j["axis"] / np.linalg.norm(j["axis"])
            ax.set("xyz", " ".join(f"{x:.6g}" for x in a))

    # 诊断
    _, T2 = fk(joints)
    print("\n=== 转动后腕部几何 ===")
    for bid in ("B7", "B8", "B9", "B10", "B11"):
        print(f"  {bid}: 帧原点 {np.round(T2[bid][:3,3]*1000,1)} mm")

    # R9 闭环残差：B6 -> B11 的物理销 vs 树里 B11 的帧
    loops = ET.fromstring(open(os.path.join(V2, "loops.yaml"), encoding="utf-8").read()) \
        if False else None
    import json
    lo = json.load(open(os.path.join(V2, "loops.yaml"), encoding="utf-8"))
    mp = json.load(open(os.path.join(V2, "mapping.json"), encoding="utf-8"))
    o_b0 = np.array(mp["B0"]["origin_mm"])
    o_root = T2[root][:3, 3] * 1000
    for lp in lo:
        par, ch = lp["parent"], lp["child"]
        want = np.array(lp["origin_mm"]) - o_b0
        got = T2[ch][:3, 3] * 1000 - o_root
        print(f"  {lp['joint']:4s} {par}->{ch}: 树 {np.round(got,1)} 记录 {np.round(want,1)} "
              f"残差 {np.linalg.norm(got-want):.1f} mm")

    if args.dry:
        print("\n[dry] 不写文件")
        return
    out = args.out or URDF
    t.write(out, encoding="utf-8", xml_declaration=True)
    print("\n写入", out)


if __name__ == "__main__":
    main()
