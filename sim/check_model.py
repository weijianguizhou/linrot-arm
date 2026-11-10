# -*- coding: utf-8 -*-
"""在 MuJoCo 里验证 linear_arm_v2 的完整性、几何一致性、闭环约束与动力学自洽性。

用法:
    python -X utf8 tools_mujoco_check.py

分 8 节，每节打印 OK / FAIL / INFO，最后给总结论。
"""
import json
import os
import sys

import numpy as np
import mujoco

V2 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model")
MJCF = os.path.join(V2, "mjcf", "linear_arm.xml")
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")

BODIES = [f"B{i}" for i in range(13)]

fails = []
infos = []


def tag(ok, name, detail=""):
    if ok is None:
        print(f"  [INFO] {name}  {detail}")
        infos.append(name)
    else:
        print(f"  [{'OK  ' if ok else 'FAIL'}] {name}  {detail}")
        if not ok:
            fails.append(name)


def bid(m, name):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    if i < 0:
        raise KeyError(f"body {name} 不存在")
    return i


def anchors(m, d, body1, body2, a1_local, a2_local):
    """返回两个锚点的世界坐标 (m)。"""
    i1, i2 = bid(m, body1), bid(m, body2)
    p1 = d.xpos[i1] + d.xmat[i1].reshape(3, 3) @ np.asarray(a1_local, float)
    p2 = d.xpos[i2] + d.xmat[i2].reshape(3, 3) @ np.asarray(a2_local, float)
    return p1, p2


def loop_residuals(m, d, loops):
    out = []
    for lp in loops:
        p1, p2 = anchors(
            m, d, lp["parent"], lp["child"],
            np.array(lp["origin_parent_mm"], float) / 1000.0,
            np.array(lp["origin_child_mm"], float) / 1000.0,
        )
        out.append(float(np.linalg.norm(p2 - p1)) * 1000.0)
    return np.array(out)


def main():
    loops = json.load(open(os.path.join(V2, "loops.yaml"), encoding="utf-8"))
    mp = json.load(open(os.path.join(V2, "mapping.json"), encoding="utf-8"))

    print(f"MuJoCo {mujoco.__version__}")
    m = mujoco.MjModel.from_xml_path(MJCF)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    # ---------------------------------------------------------------- 1
    print("\n=== 1. 模型统计 ===")
    print(f"  nbody={m.nbody} njnt={m.njnt} nq={m.nq} nv={m.nv} "
          f"neq={m.neq} ngeom={m.ngeom} nmesh={m.nmesh}")
    print(f"  timestep={m.opt.timestep}  gravity={np.round(m.opt.gravity, 2)}  "
          f"integrator={m.opt.integrator}")
    print(f"  总质量={m.body_mass.sum():.4f} kg  "
          f"（B0 已被 fusestatic 并入 world，mapping 合计 "
          f"{sum(v['mass_kg'] for v in mp.values()):.4f} kg）")
    tag(m.njnt == 12, "树关节数 = 12", f"实际 {m.njnt}")
    tag(m.neq == 3, "闭环等式约束数 = 3", f"实际 {m.neq}")
    tag(m.nq == m.nv, "nq == nv（无四元数/自由关节）")

    # ---------------------------------------------------------------- 2
    print("\n=== 2. 正向运动学 vs CAD 站位 ===")
    b0 = np.array(mp["B0"]["origin_mm"], float)
    worst = 0.0
    for name in BODIES:
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            continue
        want = (np.array(mp[name]["origin_mm"], float) - b0) / 1000.0
        got = d.xpos[bid(m, name)]
        err = float(np.linalg.norm(got - want)) * 1000.0
        worst = max(worst, err)
    tag(worst < 0.05, "13 个刚体的 FK 站位与 CAD 一致", f"最大偏差 {worst:.3f} mm")

    # ---------------------------------------------------------------- 3
    print("\n=== 3. 闭环几何闭合（q=0） ===")
    res = loop_residuals(m, d, loops)
    for lp, r in zip(loops, res):
        print(f"  {lp['joint']:4s} {lp['parent']}->{lp['child']}  "
              f"锚点间距 {r:.4f} mm")
    tag(res.max() < 0.05, "q=0 时三个闭环完全闭合", f"最大 {res.max():.4f} mm")

    # ---------------------------------------------------------------- 4
    print("\n=== 4. 约束雅可比秩 / 有效自由度 ===")
    rows = []
    for lp in loops:
        p1, p2 = anchors(
            m, d, lp["parent"], lp["child"],
            np.array(lp["origin_parent_mm"], float) / 1000.0,
            np.array(lp["origin_child_mm"], float) / 1000.0,
        )
        J1 = np.zeros((3, m.nv))
        J2 = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, J1, None, p1, bid(m, lp["parent"]))
        mujoco.mj_jac(m, d, J2, None, p2, bid(m, lp["child"]))
        rows.append(J2 - J1)
    J = np.vstack(rows)
    s = np.linalg.svd(J, compute_uv=False) if J.size else np.zeros(0)
    rank = int(np.sum(s > max(s.max() * 1e-6, 1e-8))) if s.size else 0
    dof = m.nv - rank
    print(f"  等式约束 Jacobian {J.shape}, 奇异值 = {np.round(s, 4)}")
    print(f"  秩 = {rank}, 有效自由度 = nv - rank = {dof}")
    tag(None, f"当前模型有效自由度 = {dof}",
        "（若设计要求 3 肩 + 1 肘 + 2 腕 = 6，则缺一个肩部转轴）")

    # ---------------------------------------------------------------- 5
    print("\n=== 5. 质量矩阵 ===")
    M = np.zeros((m.nv, m.nv))
    try:
        mujoco.mj_fullM(m, d, M)          # MuJoCo >= 3.2
    except TypeError:
        mujoco.mj_fullM(m, M, d.qM)       # 旧版签名
    ev = np.linalg.eigvalsh(M)
    tag(ev.min() > 0, "质量矩阵正定", f"特征值范围 [{ev.min():.3e}, {ev.max():.3e}]")
    print(f"  条件数 = {ev.max() / ev.min():.1f}（>1e6 说明某方向惯量极小，需调 armature）")

    # ---------------------------------------------------------------- 6
    print("\n=== 6. q=0 静态重力负载（各关节需要的保持力/力矩） ===")
    qf = d.qfrc_bias.copy()
    for i in range(m.njnt):
        jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        typ = "N·m" if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE else "N"
        print(f"  {jn:4s} {typ:3s}  {qf[m.jnt_dofadr[i]]:+.4f}")
    gpe = -float(np.sum(m.body_mass[:, None] * d.xipos * m.opt.gravity[None, :]))
    print(f"  重力势能 = {gpe:.4f} J（以世界原点为零点），"
          f"整机质量 {m.body_mass.sum():.3f} kg")

    # ---------------------------------------------------------------- 7
    print("\n=== 7. 闭环动态保持测试（去重力 + 扰动静止收敛） ===")
    d2 = mujoco.MjData(m)
    m2 = mujoco.MjModel.from_xml_path(MJCF)
    m2.opt.gravity[:] = 0.0
    m2.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICIT
    m2.opt.timestep = 0.002
    d2.qpos[:] = 0.0
    # 扰动：三个直线副各 +5 mm，腕部绕 R6 转 0.05 rad
    for jn, val in (("L1", 0.005), ("L2", 0.005), ("L3", 0.005), ("R6", 0.05)):
        jid = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, jn)
        d2.qpos[m2.jnt_qposadr[jid]] += val
    mujoco.mj_forward(m2, d2)
    r0 = loop_residuals(m2, d2, loops)
    for _ in range(1500):
        mujoco.mj_step(m2, d2)
    r1 = loop_residuals(m2, d2, loops)
    print(f"  扰动后 初始残差 = {np.round(r0, 2)} mm")
    print(f"  1.5e3 步后 残差 = {np.round(r1, 3)} mm")
    tag(r1.max() < 0.5, "闭环约束能把机构拉回闭合", f"收敛后最大 {r1.max():.3f} mm")

    # ---------------------------------------------------------------- 8
    print("\n=== 8. 碰撞设置 ===")
    contype = np.unique(m.geom_contype)
    tag(None, f"geom contype = {contype.tolist()}",
        "全 0 表示不做碰撞检测：适合运动学/动力学校核，但无法检查零件互相穿插")

    # ---------------------------------------------------------------- 9
    print("\n=== 9. 来源文件 ===")
    tag(os.path.exists(MJCF), "MJCF 存在", MJCF)
    tag(os.path.exists(URDF), "URDF 存在", URDF)

    print("\n" + "=" * 60)
    if fails:
        print(f"总结论: 有 {len(fails)} 项 FAIL -> {fails}")
    else:
        print("总结论: 全部通过")
    print("=" * 60)
    print("\n可视化：python -m mujoco.viewer --mjcf=\"%s\"" % MJCF)
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
