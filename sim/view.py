# -*- coding: utf-8 -*-
"""用 MuJoCo 交互界面查看 linear_arm_v2。

默认关掉重力：模型里没有电机，一开重力整条手臂会直接垂下去，
关掉之后手臂会停在当前姿态，用鼠标拖动就能摆姿势。

默认铺 MuJoCo 例程里那套经典背景（渐变天空 + 棋盘地面）。它只加在
渲染层，不参与碰撞，也不进动力学。

用法:
    python -X utf8 tools_view.py              # 关重力，经典背景
    python -X utf8 tools_view.py --no-bg      # 不要背景，纯色底
    python -X utf8 tools_view.py --gravity    # 真实重力，看它会怎么塌
    python -X utf8 tools_view.py --urdf       # 用 URDF 看（没有闭环约束）
    python -X utf8 tools_view.py --check      # 只检查能不能加载，不开界面
"""
import argparse
import os
import sys

import mujoco
import numpy as np

V2 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model")
MJCF = os.path.join(V2, "mjcf", "linear_arm.xml")
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")


def _mesh_zmin(m, d):
    """q=0 姿态下所有网格的最低点，棋盘地面铺在它下面。"""
    zmin = np.inf
    for gid in range(m.ngeom):
        if m.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[gid]
        ath, num = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        vert = m.mesh_vert[ath:ath + num]
        rot = d.geom_xmat[gid].reshape(3, 3)
        zmin = min(zmin, float((vert @ rot.T + d.geom_xpos[gid])[:, 2].min()))
    return zmin if np.isfinite(zmin) else 0.0


def _with_classic_background(path, m, d):
    """给模型套上 MuJoCo 官方例程同款的天空盒和棋盘地面。"""
    spec = mujoco.MjSpec.from_file(path)
    spec.add_texture(
        name="skybox", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.3, 0.5, 0.7], rgb2=[0.0, 0.0, 0.0],
        width=512, height=3072)
    spec.add_texture(
        name="groundplane", type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        mark=mujoco.mjtMark.mjMARK_EDGE,
        rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3],
        markrgb=[0.8, 0.8, 0.8], width=300, height=300)
    # MjSpec 的 textures 从 role 0 起填，槽位 1 才是 RGB
    spec.add_material(
        name="groundplane", textures=["", "groundplane"],
        texuniform=True, texrepeat=[5, 5], reflectance=0.2)
    floor = spec.worldbody.add_geom(
        name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05], pos=[0, 0, _mesh_zmin(m, d) - 0.01],
        contype=0, conaffinity=0, group=0)
    floor.material = "groundplane"

    m_bg = spec.compile()
    if mujoco.mj_name2id(m_bg, mujoco.mjtObj.mjOBJ_MATERIAL, "groundplane") < 0:
        return None
    return m_bg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gravity", action="store_true",
                    help="保留 9.81 重力（默认关掉，方便静止观察）")
    ap.add_argument("--urdf", action="store_true", help="加载 URDF 而不是 MJCF")
    ap.add_argument("--no-bg", action="store_true",
                    help="不要经典背景，保持纯色底")
    ap.add_argument("--check", action="store_true", help="只校验加载，不打开窗口")
    args = ap.parse_args()

    path = URDF if args.urdf else MJCF
    if not os.path.exists(path):
        sys.exit(f"找不到模型: {path}")

    m = mujoco.MjModel.from_xml_path(path)
    if not args.gravity:
        m.opt.gravity[:] = 0.0
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    m_view, d_view, bg = m, d, False
    if not args.no_bg:
        m_bg = _with_classic_background(path, m, d)
        if m_bg is None:
            print("INFO   : 这个模型加不了经典背景，保持纯色底")
        else:
            if not args.gravity:
                m_bg.opt.gravity[:] = 0.0
            m_view, d_view = m_bg, mujoco.MjData(m_bg)
            mujoco.mj_forward(m_view, d_view)
            bg = True

    print(f"模型   : {path}")
    print(f"版本   : MuJoCo {mujoco.__version__}")
    print(f"规模   : nbody={m.nbody} njnt={m.njnt} nq={m.nq} neq={m.neq} "
          f"ngeom={m.ngeom}")
    print(f"质量   : {m.body_mass.sum():.4f} kg")
    print(f"重力   : {m.opt.gravity}  "
          f"({'真实重力' if args.gravity else '已关闭'})")
    print(f"背景   : {'经典（渐变天空 + 棋盘地面）' if bg else '纯色'}")
    print("关节   : " + ", ".join(
        mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)))
    if not args.urdf and m.neq:
        print("闭环   : " + ", ".join(
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, i) for i in range(m.neq)))

    if args.check:
        print("\n加载正常（--check 模式，不打开界面）")
        return 0

    print("""
操作提示
--------
  左键拖动     旋转视角
  右键拖动     平移
  滚轮         缩放
  空格         暂停 / 继续
  双击刚体     选中单个零件
  Ctrl + 右键拖动   直接拖拽刚体摆姿态（Perturb）
  左侧 Physics 面板  可随时改重力、时间步
  左侧 Rendering 面板 可切换线框、透明、坐标系
""")
    try:
        from mujoco import viewer as _viewer
    except Exception as e:  # noqa: BLE001
        sys.exit(f"打不开 viewer（{e}）。可以用命令行的 python -m mujoco.viewer 试试。")

    _viewer.launch(m_view, d_view)
    return 0


if __name__ == "__main__":
    sys.exit(main())
