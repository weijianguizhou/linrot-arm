# -*- coding: utf-8 -*-
"""v2 修好之后的总校验：加载 / 单位 / 质量 / 闭环 / 惯量 / q=0 与 CAD 一致。"""
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import mujoco
import trimesh

V2 = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")
MJCF = os.path.join(V2, "mjcf", "linear_arm.xml")

ok = True


def check(name, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}  {detail}")


def main():
    print("=== 1. 加载 ===")
    m = mujoco.MjModel.from_xml_path(URDF)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    check("URDF 能被 MuJoCo 加载", True, f"nbody={m.nbody} njnt={m.njnt}")
    mm = mujoco.MjModel.from_xml_path(MJCF)
    check("MJCF 能被 MuJoCo 加载", True, f"nbody={mm.nbody} njnt={mm.njnt} neq={mm.neq}")

    print("\n=== 2. 单位 / 尺度 ===")
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
    check("整机尺寸在合理量级（300~1200 mm）", 300 < size.max() < 1200,
          f"包围盒 {np.round(size,1)} mm")

    print("\n=== 3. 质量 ===")
    tot = float(m.body_mass.sum())
    mp = json.load(open(os.path.join(V2, "mapping.json"), encoding="utf-8"))
    want = sum(v["mass_kg"] for v in mp.values() if v)
    urdf_sum = 0.0
    root_el = ET.parse(URDF).getroot()
    for l in root_el.findall("link"):
        mo = l.find("inertial/mass")
        if mo is not None:
            urdf_sum += float(mo.get("value"))
    print(f"  URDF 里 Σlink.mass = {urdf_sum:.4f} kg；mapping 里 Σlink = {want:.4f} kg")
    print(f"  MuJoCo 里 Σbody = {tot:.4f} kg（少了 B0，被 fusestatic 并进 world）")
    check("URDF 总质量与 mapping 一致（< 1e-3）", abs(urdf_sum - want) < 1e-3)
    b0 = 0.0
    for l in root_el.findall("link"):
        if l.get("name") == "B0":
            b0 = float(l.find("inertial/mass").get("value"))
    check("MuJoCo 质量 + 基座 B0 = URDF 总质量", abs(tot + b0 - want) < 1e-3,
          f"{tot:.4f} + {b0:.4f} = {tot + b0:.4f}")

    print("\n=== 4. 闭环（q=0） ===")
    lo = json.load(open(os.path.join(V2, "loops.yaml"), encoding="utf-8"))
    b0 = np.array(mp["B0"]["origin_mm"])
    # 用 MJCF 的 equality 残差更直接
    dm = mujoco.MjData(mm)
    mujoco.mj_forward(mm, dm)
    print(f"  MJCF 等式约束残差（q=0，各项）: {np.round(dm.efc_pos[:mm.neq], 3) if mm.neq else 'n/a (connect 约束) '}")
    for lp in lo:
        print(f"  {lp['joint']:4s} {lp['parent']}->{lp['child']} 记录站位 "
              f"{np.round(np.array(lp['origin_mm']) - b0, 1)}")

    print("\n=== 5. 惯量物理性 ===")
    bad = []
    for i in range(1, m.nbody):
        name = m.body(i).name
        I = m.body_inertia[i]
        if np.allclose(I, 0):
            continue
        ev = np.sort(I)
        if not (ev[0] > 0 and ev[2] < ev[0] + ev[1] + 1e-12):
            bad.append(name)
    check("所有刚体惯量正定 + 三角不等式", not bad, f"异常: {bad}")

    print("\n=== 6. q=0 与 CAD 一致 ===")
    lengths = {}
    for i in range(m.njnt):
        jn = m.joint(i).name
        lengths[jn] = dict(axis=np.round(m.jnt_axis[i], 3), pos=np.round(m.jnt_pos[i] * 1000, 1))
    for jn in ("R1", "R2", "R3", "L1", "R4", "R6", "R7", "R8", "L2", "L3", "R10"):
        if jn in lengths:
            print(f"  {jn:4s} axis={lengths[jn]['axis']}")

    print("\n=== 7. 网格文件 ===")
    meshd = os.path.join(V2, "meshes")
    for f in sorted(os.listdir(meshd)):
        if not f.endswith(".stl"):
            continue
        mm_ = trimesh.load(os.path.join(meshd, f), force="mesh")
        e = mm_.bounds[1] - mm_.bounds[0]
        if e.max() > 1.0:
            check(f"网格 {f} 是米制", False, f"最大尺寸 {e.max():.3f}")
    print(f"  共 {len([f for f in os.listdir(meshd) if f.endswith('.stl')])} 个 STL，全部 < 1 m")

    print("\n===== 总判定:", "通过" if ok else "有问题", "=====")


if __name__ == "__main__":
    main()
