# -*- coding: utf-8 -*-
"""修 v2 URDF 的加载问题（可重复执行，带哨兵文件防止重复缩放）。

修三处：
  1) 网格单位：STL 原来是 mm，URDF 约定 m -> 全部缩到 m
  2) 网格路径：filename="meshes/..." 相对 URDF 文件解析会失败 -> "../meshes/..."
  3) 根刚体：MuJoCo 会把根 link 并进 world，B0 的 0.70 kg 会丢
     -> 加一个 base_link + fixed 关节，B0 变成普通刚体

同时把 mjcf/linear_arm.xml 的 meshdir 指到 ../meshes、scale 改回 1。
"""
import json
import os
import re
import shutil

import numpy as np
import trimesh

V2 = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2"
MESH = os.path.join(V2, "meshes")
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")
MJCF = os.path.join(V2, "mjcf", "linear_arm.xml")
SENTINEL = os.path.join(MESH, ".in_meters")


def scale_meshes():
    if os.path.exists(SENTINEL):
        print("[1] 网格已是米制，跳过")
        return
    files = sorted(f for f in os.listdir(MESH) if f.lower().endswith(".stl"))
    bak = os.path.join(V2, "meshes_mm_backup")
    os.makedirs(bak, exist_ok=True)
    for f in files:
        p = os.path.join(MESH, f)
        m = trimesh.load(p, force="mesh")
        m.apply_scale(0.001)
        m.export(p)
        shutil.copy2(p, os.path.join(bak, f))  # 备份（此时已是 m）
    json.dump({"scaled_by": 0.001, "files": files},
              open(SENTINEL, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[1] {len(files)} 个 STL 已缩到米制，原文件备份在 meshes_mm_backup/")

    # 把备份还原成真正的 mm 版本，命名区分清楚
    for f in files:
        p = os.path.join(MESH, f)
        m = trimesh.load(p, force="mesh")
        m.apply_scale(1000.0)
        m.export(os.path.join(bak, f))


def fix_urdf():
    s = open(URDF, encoding="utf-8").read()
    n = 0

    def rep(mo):
        nonlocal n
        n += 1
        return 'filename="../meshes/'

    s2 = re.sub(r'filename="(?:\.\./)?meshes/', rep, s)

    if '<link name="base_link"' not in s2:
        # 在 <robot ...> 之后插入 base_link + 固定关节
        m = re.search(r'(<robot[^>]*>\s*)', s2)
        ins = ('  <link name="base_link"/>\n'
               '  <!-- 基座：让 B0 成为真实刚体，否则 MuJoCo 会把根 link 并进 world、丢掉 0.70 kg 惯量 -->\n'
               '  <joint name="base_joint" type="fixed">\n'
               '    <parent link="base_link"/>\n'
               '    <child link="B0"/>\n'
               '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
               '  </joint>\n')
        s2 = s2[:m.end(1)] + ins + s2[m.end(1):]
        print("[2] 加了 base_link + base_joint")
    else:
        print("[2] base_link 已存在，跳过")

    open(URDF, "w", encoding="utf-8").write(s2)
    print(f"[3] 网格路径改了 {n} 处")


def fix_mjcf():
    if not os.path.exists(MJCF):
        print("[4] 没有 mjcf，跳过")
        return
    s = open(MJCF, encoding="utf-8").read()
    s2 = s.replace('meshdir="meshes"', 'meshdir="../meshes"')
    s2 = s2.replace('scale="0.001 0.001 0.001"', 'scale="1 1 1"')
    s2 = s2.replace('<mesh name="base_link', '<mesh name="unused_base')
    open(MJCF, "w", encoding="utf-8").write(s2)
    print("[4] mjcf 指向 ../meshes，scale=1")


def verify():
    import mujoco
    m = mujoco.MjModel.from_xml_path(URDF)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    pts = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = m.mesh_vert[va:va + vn]
        pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    P = np.concatenate(pts)
    size = (P.max(0) - P.min(0)) * 1000
    print("\n=== 验证 ===")
    print(f"nbody={m.nbody} njnt={m.njnt} 总质量={m.body_mass.sum():.4f} kg")
    print(f"整机包围盒 {np.round(size,1)} mm")
    names = [m.body(i).name for i in range(m.nbody)]
    print("刚体:", names)
    mass = {m.body(i).name: round(float(m.body_mass[i]), 4) for i in range(m.nbody)}
    print("质量:", mass)
    return size


if __name__ == "__main__":
    scale_meshes()
    fix_urdf()
    fix_mjcf()
    verify()
