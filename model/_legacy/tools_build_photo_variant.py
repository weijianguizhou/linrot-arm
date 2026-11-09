# -*- coding: utf-8 -*-
"""按照片（手臂动力学模型.png）重排腕部（第二版）。

第一版手填坐标 -> 模组顶端埋进支架、B8 悬空。这一版所有位置都由
「贴合条件」从实测包围盒算出来：

  模组顶端   = 支架下沿 + 5 mm（略插进支架）
  模组前沿   = 支架 +X 面 + 半厚 - 2 mm（贴支架正面）
  活塞杆顶端 = 模组下端 + 25 mm（插进模组）
  端板顶面   = 杆底 - 10 mm（插进端板）
  十字轴     = 两模组顶端之间，y 取中
  两列模组   = 沿 y 并排，间距 5 mm

上半臂 B0..B5 完全不动。做法仍是「定世界位姿 -> 反解关节 origin/axis」。
"""
from __future__ import annotations

import os
import shutil

import numpy as np
import trimesh
import xml.etree.ElementTree as ET

SRC = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2"
DST = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2_photo"

# 注意：几何全是米，这几个贴合量要换算成米
GAP_TOP = -0.004     # 模组顶端插进支架下沿 4 mm
GAP_FACE = 0.014     # 保留：模组贴支架正面的压入量（当前布局未用）
PROTRUDE = 0.015     # 零位（行程=0）时活塞杆伸出模组的长度：基本收回
PLATE_IN = 0.005     # 端板与杆端头的搭接量
Y_PITCH = 0.005      # 两列模组之间的间隙

# 本地 +X -> 世界 +Z：模组立起来，杆朝下
R_MOD = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])


def rpy_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_rpy(R):
    sy = float(np.clip(-R[2, 0], -1, 1))
    p = np.arcsin(sy)
    if abs(abs(sy) - 1.0) < 1e-9:
        return 0.0, p, np.arctan2(-R[0, 1], R[1, 1])
    return np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])


def main():
    tree = ET.parse(os.path.join(SRC, "urdf", "linear_arm.urdf"))
    root = tree.getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        ax = j.find("axis")
        joints[j.get("name")] = dict(
            el=j,
            xyz=np.array([float(x) for x in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3),
            rpy=np.array([float(x) for x in o.get("rpy", "0 0 0").split()]) if o is not None else np.zeros(3),
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            axis=np.array([float(x) for x in ax.get("xyz", "1 0 0").split()]) if ax is not None else None,
            type=j.get("type"))

    kids = {j["child"] for j in joints.values()}
    links = {j["parent"] for j in joints.values()} | kids
    rname = [l for l in links if l not in kids][0]
    T = {rname: np.eye(4)}
    ch = True
    while ch:
        ch = False
        for j in joints.values():
            if j["parent"] in T and j["child"] not in T:
                M = np.eye(4)
                M[:3, :3] = rpy_matrix(*j["rpy"])
                M[:3, 3] = j["xyz"]
                T[j["child"]] = T[j["parent"]] @ M
                ch = True

    loc, wbb, wc = {}, {}, {}
    for l in root.findall("link"):
        name = l.get("name")
        me = l.find("visual/geometry/mesh")
        if me is None:
            continue
        vis = l.find("visual/origin")
        off = np.array([float(x) for x in vis.get("xyz", "0 0 0").split()]) if vis is not None else np.zeros(3)
        mm = trimesh.load(os.path.normpath(os.path.join(SRC, "urdf", me.get("filename"))), force="mesh")
        mm.apply_translation(off)
        loc[name] = mm
        b = mm.bounds
        wbb[name] = (T[name][:3, :3] @ b.T).T + T[name][:3, 3]
        wc[name] = (T[name][:3, :3] @ ((b[0] + b[1]) / 2)) + T[name][:3, 3]

    # ---- 从 B5 的实测包围盒推布局
    b5 = wbb["B5"]
    x_face, z_bot = b5[1, 0], b5[0, 2]
    y_c = (b5[0, 1] + b5[1, 1]) / 2

    def half_ext(name, rotated):
        """局部包围盒的半径；rotated=True 表示要按 Ry(-90°) 换轴（本地X->世界Z）"""
        e = (loc[name].bounds[1] - loc[name].bounds[0]) / 2
        return (e[2], e[1], e[0]) if rotated else (e[0], e[1], e[2])

    mhx, mhy, mhz = half_ext("B7", True)      # 模组：长轴在本地 X
    rhx, rhy, rhz = half_ext("B9", True)      # 活塞杆：同样横躺
    phx, phy, phz = half_ext("B11", False)    # 端板：不转

    x_mod = (b5[0, 0] + b5[1, 0]) / 2      # 就在支架正下方，不在前面
    z_top = z_bot - GAP_TOP
    z_mod = z_top - mhz
    y1 = y_c + (mhy + Y_PITCH / 2)
    y2 = y_c - (mhy + Y_PITCH / 2)
    z_mod_bot = z_top - 2 * mhz
    z_rod_bot = z_mod_bot - PROTRUDE          # 零位：杆尾几乎与模组下端面齐平
    z_rod_top = z_rod_bot + 2 * rhz
    z_rod = (z_rod_top + z_rod_bot) / 2
    z_plat = z_rod_bot + PLATE_IN - phz

    TARGET = {
        "B6":  (np.eye(3), np.array([x_mod, y_c, z_top])),
        "B7":  (R_MOD, np.array([x_mod, y1, z_mod])),
        "B8":  (R_MOD, np.array([x_mod, y2, z_mod])),
        "B9":  (R_MOD, np.array([x_mod, y1, z_rod])),
        "B10": (R_MOD, np.array([x_mod, y2, z_rod])),
        "B11": (np.eye(3), np.array([x_mod, y_c, z_plat])),
    }
    print("=== 布局（URDF 根坐标系，mm）===")
    print(f"  支架 +X 面 {x_face*1000:8.1f}   下沿 {z_bot*1000:8.1f}   中心 y {y_c*1000:7.1f}")
    for k, (R, c) in TARGET.items():
        print(f"  {k:4s} 中心 {np.round(c*1000,1)}")

    Tnew = {k: v.copy() for k, v in T.items()}
    for link, (R, c_new) in TARGET.items():
        P = np.eye(4)
        P[:3, :3] = R
        P[:3, 3] = c_new - R @ wc[link]
        Tnew[link] = P @ T[link]

    for jn, v in joints.items():
        p, c = v["parent"], v["child"]
        if p not in Tnew or c not in Tnew:
            continue
        O = np.linalg.inv(Tnew[p]) @ Tnew[c]
        v["xyz"] = O[:3, 3]
        v["rpy"] = np.array(matrix_rpy(O[:3, :3]))
        if v["axis"] is not None:
            if jn in ("R6", "R7", "R8", "R9", "R10", "R11"):
                aw = np.array([0.0, 1.0, 0.0])
            elif jn in ("L2", "L3"):
                aw = np.array([0.0, 0.0, 1.0])
            else:
                aw = None
            if aw is not None:
                v["axis"] = Tnew[c][:3, :3].T @ aw
                v["axis"] /= np.linalg.norm(v["axis"])

    for jn, v in joints.items():
        el = v["el"]
        o = el.find("origin")
        o.set("xyz", " ".join(f"{x:.6g}" for x in v["xyz"]))
        o.set("rpy", " ".join(f"{x:.6g}" for x in v["rpy"]))
        ax = el.find("axis")
        if ax is not None and v["axis"] is not None:
            ax.set("xyz", " ".join(f"{x:.6g}" for x in v["axis"]))

    os.makedirs(os.path.join(DST, "urdf"), exist_ok=True)
    mdst = os.path.join(DST, "meshes")
    if not os.path.isdir(mdst):
        shutil.copytree(os.path.join(SRC, "meshes"), mdst)
    out = os.path.join(DST, "urdf", "linear_arm_photo.urdf")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print("写入", out)

    print("\n=== 新包围盒（mm）===")
    for k in ("B5", "B6", "B7", "B8", "B9", "B10", "B11"):
        b = ((Tnew[k][:3, :3] @ loc[k].bounds.T).T + Tnew[k][:3, 3]) * 1000
        print(f"  {k:4s} X[{b[0,0]:7.1f},{b[1,0]:7.1f}] "
              f"Y[{b[0,1]:7.1f},{b[1,1]:7.1f}] Z[{b[0,2]:7.1f},{b[1,2]:7.1f}]")


if __name__ == "__main__":
    main()
