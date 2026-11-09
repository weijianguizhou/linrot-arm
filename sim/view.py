# -*- coding: utf-8 -*-
"""用 MuJoCo 交互界面查看 linear_arm_v2。

默认关掉重力：模型里没有电机，一开重力整条手臂会直接垂下去，
关掉之后手臂会停在当前姿态，用鼠标拖动就能摆姿势。

用法:
    python -X utf8 tools_view.py              # 关重力，适合看结构 / 摆姿态
    python -X utf8 tools_view.py --gravity    # 真实重力，看它会怎么塌
    python -X utf8 tools_view.py --urdf       # 用 URDF 看（没有闭环约束）
    python -X utf8 tools_view.py --check      # 只检查能不能加载，不开界面
"""
import argparse
import os
import sys

import mujoco

V2 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model")
MJCF = os.path.join(V2, "mjcf", "linear_arm.xml")
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gravity", action="store_true",
                    help="保留 9.81 重力（默认关掉，方便静止观察）")
    ap.add_argument("--urdf", action="store_true", help="加载 URDF 而不是 MJCF")
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

    print(f"模型   : {path}")
    print(f"版本   : MuJoCo {mujoco.__version__}")
    print(f"规模   : nbody={m.nbody} njnt={m.njnt} nq={m.nq} neq={m.neq} "
          f"ngeom={m.ngeom}")
    print(f"质量   : {m.body_mass.sum():.4f} kg")
    print(f"重力   : {m.opt.gravity}  "
          f"({'真实重力' if args.gravity else '已关闭'})")
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

    _viewer.launch(m, d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
