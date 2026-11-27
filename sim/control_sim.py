# -*- coding: utf-8 -*-
"""1 kHz 运控仿真：位置执行器 + 重力前馈 + 关节/笛卡尔跟踪。

模型已配 6 个位置执行器（`act_R1/R2/R3` 肩、`act_L1/L2/L3` 肘与腕电缸），
物理步长 1 ms。本脚本在 Python 侧按 1 kHz 下发目标，并做三件事：

1. 重力前馈：每步把 `qfrc_bias` 里 6 个驱动 DOF 的分量加到 `qfrc_applied`，
   补偿位置伺服在大重力负载下的稳态误差。
2. 关节模式：给驱动关节阶跃 / 正弦目标，看位置环跟踪。
3. 笛卡尔模式：以 B12 原点为末端控制点，用“约束零空间 + 执行器映射”的
   约化雅可比做阻尼最小二乘 IK，让末端走圆轨迹。

约束（肘四连杆、腕部双缸）由 MuJoCo 的等式约束求解器闭合；IK 只在
6 个执行器坐标上积分，不直接改被动关节。

用法:
    python -X utf8 sim/control_sim.py                     # 默认跑 step
    python -X utf8 sim/control_sim.py --mode sine
    python -X utf8 sim/control_sim.py --mode cartesian --viewer
    python -X utf8 sim/control_sim.py --mode all --plot

输出:
    sim/logs/control_sim_<mode>.csv    每步记录
    sim/logs/control_sim_<mode>.png    --plot 时生成
    末行: 总结论: 全部通过 / 有 N 项 FAIL
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(os.path.dirname(HERE), "model")
MJCF = os.path.join(MODEL, "mjcf", "linear_arm.xml")
LOOPS = os.path.join(MODEL, "loops.yaml")
LOG_DIR = os.path.join(HERE, "logs")

ACTUATORS = ["act_R1", "act_R2", "act_R3", "act_L1", "act_L2", "act_L3"]
JOINTS = ["R1", "R2", "R3", "L1", "L2", "L3"]
END_BODY = "B12"

# q=0 时肘部接近死点：L1 负向回缩可以平稳弯肘，正向推到 +11 mm 就顶住。
# 所以统一从这点“预弯”姿态起步，避开奇异位形。
Q_START = np.array([0.0, 0.0, 0.0, -0.03, 0.0, 0.0])
RAMP_TIME = 1.0          # 从 q=0 平滑过渡到 Q_START 的时间 (s)
RATE_REV = 2.0           # 关节目标限速 (rad/s)
RATE_SLIDE = 0.1         # 电缸目标限速 (m/s)
IK_GAIN = 5.0            # 笛卡尔 IK 闭环增益 (1/s)，再高会和位置伺服打架


def tag(ok, name, detail=""):
    print(f"  [{'OK  ' if ok else 'FAIL'}] {name}  {detail}")
    return bool(ok)


def anchors(m, d, lp):
    i1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["parent"])
    i2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["child"])
    a1 = np.asarray(lp["origin_parent_mm"], float) / 1000.0
    a2 = np.asarray(lp["origin_child_mm"], float) / 1000.0
    p1 = d.xpos[i1] + d.xmat[i1].reshape(3, 3) @ a1
    p2 = d.xpos[i2] + d.xmat[i2].reshape(3, 3) @ a2
    return p1, p2


def loop_residual_mm(m, d, loops):
    out = []
    for lp in loops:
        p1, p2 = anchors(m, d, lp)
        out.append(float(np.linalg.norm(p2 - p1)) * 1000.0)
    return np.array(out)


class ArmSim:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(MJCF)
        self.d = mujoco.MjData(self.m)
        self.loops = json.load(open(LOOPS, encoding="utf-8"))

        m = self.m
        self.act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                    for n in ACTUATORS]
        self.jnt = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
                    for n in JOINTS]
        assert all(i >= 0 for i in self.act) and all(i >= 0 for i in self.jnt), \
            "执行器或关节名不匹配，检查 MJCF"
        self.qadr = np.array([m.jnt_qposadr[i] for i in self.jnt])
        self.dadr = np.array([m.jnt_dofadr[i] for i in self.jnt])
        self.body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, END_BODY)

        self.slide = np.array(
            [m.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE for i in self.jnt])
        self.rate = np.where(self.slide, RATE_SLIDE, RATE_REV) * m.opt.timestep
        # 5 维虚拟输入 u=[R1,R2,R3,L1,w]，w=L2-L3 是腕部差动行程；
        # L2=+w/2、L3=-w/2，锁死 L2+L3=0。P 把 u 的增量映射回 6 个执行器。
        self.P = np.zeros((6, 5))
        self.P[np.arange(4), np.arange(4)] = 1.0
        self.P[4, 4], self.P[5, 4] = 0.5, -0.5
        self.u_rate = np.array([RATE_REV, RATE_REV, RATE_REV,
                                RATE_SLIDE, 2.0 * RATE_SLIDE]) * m.opt.timestep

        lo = np.array([m.actuator_ctrlrange[a, 0] for a in self.act])
        hi = np.array([m.actuator_ctrlrange[a, 1] for a in self.act])
        self.ctrl_lo, self.ctrl_hi = lo, hi
        self.force_max = np.array([m.actuator_forcerange[a, 1] for a in self.act])
        self.jnt_rng = m.jnt_range[self.jnt]

        mujoco.mj_forward(m, self.d)
        self.rows = []

    # ---------------------------------------------------------- 基础量
    def q_act(self):
        return self.d.qpos[self.qadr].copy()

    def set_ctrl(self, q):
        q = np.clip(q, self.ctrl_lo, self.ctrl_hi)
        self.d.ctrl[self.act] = q

    def gravity_ff(self):
        """把 6 个驱动 DOF 的重力/科氏项直接前馈，其余置零。"""
        d = self.d
        d.qfrc_applied[:] = 0.0
        d.qfrc_applied[self.dadr] = d.qfrc_bias[self.dadr]

    def task_jac(self):
        Jp = np.zeros((3, self.m.nv))
        p = self.d.xpos[self.body]
        mujoco.mj_jac(self.m, self.d, Jp, None, p, self.body)
        return Jp

    def nullspace(self):
        """约束雅可比的零空间 N 和执行器映射的伪逆 pinv(A)。"""
        m, d = self.m, self.d
        rows = []
        for lp in self.loops:
            p1, p2 = anchors(m, d, lp)
            J1 = np.zeros((3, m.nv))
            J2 = np.zeros((3, m.nv))
            i1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["parent"])
            i2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["child"])
            mujoco.mj_jac(m, d, J1, None, p1, i1)
            mujoco.mj_jac(m, d, J2, None, p2, i2)
            rows.append(J2 - J1)
        Jc = np.vstack(rows)
        _, s, Vt = np.linalg.svd(Jc, full_matrices=True)
        # 阈值不能太紧：闭环约束有 0.03 mm 级的求解残差时，第 9 个奇异值会
        # 从 0 抬到 1e-5 量级；用 1e-6 会把真实零空间切掉一维（DOF 10 -> 9），
        # 执行器映射 A 随之掉秩，末端 IK 直接失控。
        rank = int(np.sum(s > s.max() * 1e-3))
        N = Vt[rank:].T
        E = np.zeros((len(self.act), m.nv))
        E[np.arange(len(self.act)), self.dadr] = 1.0
        A = E @ N
        return N, np.linalg.pinv(A, rcond=1e-6), rank

    # ---------------------------------------------------------- 求解器
    def ik_step(self, p_ref, q_ref, p_dot=None):
        """阻尼最小二乘，返回新的执行器坐标目标。

        err 是位置误差（m），J^+ err 是“一步到位”的关节增量；直接拿它
        当每毫秒的增量等于增益 1000/s，必然积分饱和。这里乘 IK_GAIN*dt。
        另外把参考速度 p_dot 前馈进去，否则纯比例跟踪在圆轨迹上会有
        v/K 的相位滞后（40 mm 圆、0.15 Hz、K=5 时约 7.5 mm）。
        """
        N, pinvA, _ = self.nullspace()
        Jred = self.task_jac() @ N @ pinvA          # 3 x 6
        err = p_ref - self.d.xpos[self.body]
        cmd = IK_GAIN * err
        if p_dot is not None:
            cmd = cmd + p_dot
        if np.linalg.norm(err) < 5e-5 and (p_dot is None or
                                           np.linalg.norm(p_dot) < 1e-6):
            return q_ref.copy()
        lam = 0.01                                  # 阻尼 (m)
        Jv = Jred @ self.P                          # 3 x 5，对虚拟输入
        du = self.m.opt.timestep * (Jv.T @ np.linalg.solve(
            Jv @ Jv.T + (lam * lam) * np.eye(3), cmd))
        du = np.clip(du, -self.u_rate, self.u_rate)
        dq = self.P @ du

        # 抗饱和：目标不能跑到实际位置太前面，否则执行器顶住时误差只会越积越大
        leash = np.where(self.slide, 0.005, 0.05)
        q_act = self.q_act()
        q_new = np.clip(q_ref + dq, q_act - leash, q_act + leash)
        q_new = np.clip(q_new, self.ctrl_lo, self.ctrl_hi)

        # 限幅可能破坏对称性；把共模（L2+L3）压回 0，保留差动行程 w
        c = 0.5 * (q_new[4] + q_new[5])
        q_new[4] -= c
        q_new[5] -= c
        return q_new

    def step(self):
        self.gravity_ff()
        mujoco.mj_step(self.m, self.d)

    def record(self, t, q_ref, q_des, p_ref=None):
        d = self.d
        res = loop_residual_mm(self.m, self.d, self.loops)
        p_act = d.xpos[self.body].copy()
        q_now = d.qpos[self.qadr]
        # 不能把“贴住限位”算成触发（R2 零位就是下限 0），也不能把约束求解的
        # 微小柔性算成越界：旋转允许 1e-4 rad，移动允许 1e-5 m。
        tol = np.where(self.slide, 1e-5, 1e-4)
        lim = int(np.any(q_now < self.jnt_rng[:, 0] - tol) or
                  np.any(q_now > self.jnt_rng[:, 1] + tol))
        p = np.zeros(3) if p_ref is None else p_ref
        self.rows.append(np.concatenate([
            [t], self.q_act(), q_ref, q_des, p_act, p,
            d.actuator_force[self.act], res, [lim]]))


# ---------------------------------------------------------------- 记录/统计
def save_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (["t"] +
              [f"q_{n}" for n in JOINTS] +
              [f"qref_{n}" for n in JOINTS] +
              [f"qdes_{n}" for n in JOINTS] +
              ["px", "py", "pz"] + ["pref_x", "pref_y", "pref_z"] +
              [f"f_{n}" for n in JOINTS] +
              [f"res_{i+1}" for i in range(3)] + ["limit"])
    data = np.asarray(rows)
    np.savetxt(path, data, delimiter=",", header=",".join(header),
               comments="", fmt="%.9g")


def summarize(sim, rows, mode, t_run):
    rows = np.asarray(rows)
    t = rows[:, 0]
    q = rows[:, 1:7]
    qref = rows[:, 7:13]
    qdes = rows[:, 13:19]
    p = rows[:, 19:22]
    pref = rows[:, 22:25]
    f = rows[:, 25:31]
    res = rows[:, 31:34]

    tail = (t >= t[-1] - 2.0) if mode == "cartesian" else (
        t >= t[-1] - 0.5) if mode == "step" else (t >= t[-1] - 4.0)
    if mode == "cartesian":
        err = np.linalg.norm(pref - p, axis=1)
        final = err[tail].mean()
        rms = float(np.sqrt(np.mean(err[tail] ** 2)))
        print(f"  末端位置误差  尾部均值 {final*1000:.2f} mm，"
              f"RMS {rms*1000:.2f} mm，峰值 {err.max()*1000:.2f} mm")
        ok = tag(final < 0.005 and rms < 0.010, "B12 笛卡尔跟踪",
                 f"尾部均值 {final*1000:.2f} mm / RMS {rms*1000:.2f} mm")
    else:
        per = q[:, :] - qdes[:, :]
        final = np.abs(per[tail]).max(axis=0)
        rms = np.sqrt(np.mean(per[tail] ** 2, axis=0))
        for i, n in enumerate(JOINTS):
            unit = "m" if sim.slide[i] else "rad"
            print(f"  {n:3s} 尾段误差  最大 {final[i]:.5f} {unit}  "
                  f"RMS {rms[i]:.5f} {unit}")
        tol = np.where(sim.slide, 1e-3, 1e-2)
        ok = tag(bool(np.all(final < tol)), "关节跟踪",
                 "容差 " + " / ".join(
                     f"{n}<{v:g}" for n, v in zip(JOINTS, tol)))

    print(f"  闭环残差      最大 {res.max():.4f} mm，"
          f"末值 {np.round(res[-1], 4)} mm")
    ok &= tag(res.max() < 0.5, "闭环约束保持闭合", f"最大 {res.max():.4f} mm")

    peak = np.abs(f).max(axis=0)
    sat = np.mean(np.abs(f) >= 0.99 * sim.force_max[None, :], axis=0)
    print("  执行器峰值    " + ", ".join(
        f"{n}={peak[i]:.1f}" for i, n in enumerate(JOINTS)))
    ok &= tag(bool(np.all(sat < 0.05)), "执行器未持续饱和",
              "饱和占比 " + ", ".join(
                  f"{n}={sat[i]*100:.1f}%" for i, n in enumerate(JOINTS)))

    nlim = int(rows[:, -1].sum())
    ok &= tag(nlim == 0, "未越出关节限位", f"{nlim} 步")

    # 单向行程与腕部锁和
    ok &= tag(bool(np.min(q[:, 1]) >= -1e-4), "R2 单向（>=0）",
              f"最小 {np.min(q[:, 1]):.2e} rad")
    ok &= tag(bool(np.max(q[:, 3]) <= 1e-5), "L1 单向（<=0）",
              f"最大 {np.max(q[:, 3]):.2e} m")
    s = q[:, 4] + q[:, 5]
    ok &= tag(bool(np.abs(s).max() < 1e-3), "腕部差动锁和 |L2+L3|<=1 mm",
              f"最大 {np.abs(s).max()*1000:.3f} mm")
    return ok


# ---------------------------------------------------------------- 目标生成
def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


def make_target(mode, t, q_start):
    """返回 (q_des, p_ref 或 None)。t 从 ramp 结束开始计。"""
    qd = q_start.copy()
    if mode == "step":
        # 1 / 3 / 5 s 三次阶跃，留 2 s 收敛
        if t >= 1.0:
            qd[0] += 0.20
        if t >= 3.0:
            qd[1] += 0.15
        if t >= 5.0:
            qd[2] -= 0.20
            qd[3] -= 0.005
        return qd, None

    if mode == "sine":
        f0 = 0.25
        s = np.sin(2 * np.pi * f0 * t)
        one = 0.5 * (1.0 - np.cos(2 * np.pi * f0 * t))   # 0..1 单边
        qd[0] += 0.20 * s
        qd[1] += 0.12 * one          # R2 只走正半轴
        qd[2] += 0.12 * s
        qd[3] -= 0.004 * one         # L1 只走负半轴
        w = 0.008 * s                # 腕部差动行程
        qd[4] += 0.5 * w
        qd[5] -= 0.5 * w
        return qd, None

    raise ValueError(mode)


def run_mode(mode, duration, viewer=False, csv_path=None, plot=False):
    print(f"\n=== mode: {mode} ===")
    sim = ArmSim()
    m, d = sim.m, sim.d
    dt = m.opt.timestep

    q_ref = np.zeros(6)                 # 从零位起步
    q_des = np.zeros(6)
    mujoco.mj_forward(m, d)

    # ---- 1) 平滑过渡到预弯姿态
    n_ramp = int(RAMP_TIME / dt)
    for k in range(n_ramp):
        q_ref = Q_START * smoothstep((k + 1) / n_ramp)
        sim.set_ctrl(q_ref)
        sim.step()
    q_ref = Q_START.copy()

    p0 = d.xpos[sim.body].copy()
    t_run = max(0.0, duration - RAMP_TIME)

    viewer_handle = None
    if viewer:
        from mujoco import viewer as _viewer
        viewer_handle = _viewer.launch_passive(m, d)
        wall0 = time.time()

    # ---- 2) 主循环（此时 d.time ≈ RAMP_TIME）
    n = int(t_run / dt)
    last_sync = 0.0
    trim = np.zeros(6)                  # 积分修正，补位置伺服的稳态误差
    trim_lim = np.where(sim.slide, 0.005, 0.01)
    ki = 2.0                            # 1/s
    for k in range(n):
        t = k * dt
        if mode == "cartesian":
            f0, radius = 0.15, 0.04
            th = 2 * np.pi * f0 * t
            p_ref = p0 + np.array([radius * (np.cos(th) - 1.0), 0.0,
                                   radius * np.sin(th)])
            p_dot = np.array([-radius * np.sin(th), 0.0,
                              radius * np.cos(th)]) * (2 * np.pi * f0)
            q_ref = sim.ik_step(p_ref, q_ref, p_dot)
            q_des = q_ref
            q_cmd = q_ref
        else:
            q_des, _ = make_target(mode, t, Q_START)
            step = np.clip(q_des - q_ref, -sim.rate, sim.rate)
            q_ref = np.clip(q_ref + step, sim.ctrl_lo, sim.ctrl_hi)

            setpoint = np.abs(q_des - q_ref) <= 1.5 * sim.rate
            if np.all(setpoint):
                trim += ki * (q_des - sim.q_act()) * dt
                trim = np.clip(trim, -trim_lim, trim_lim)
            q_cmd = np.clip(q_ref + trim, sim.ctrl_lo, sim.ctrl_hi)
            c = 0.5 * (q_cmd[4] + q_cmd[5])      # 锁和：L2/L3 只保留差动
            q_cmd[4] -= c
            q_cmd[5] -= c

        sim.set_ctrl(q_cmd)
        sim.step()
        sim.record(RAMP_TIME + (k + 1) * dt, q_cmd, q_des,
                   p_ref if mode == "cartesian" else None)

        if viewer_handle is not None:
            if not viewer_handle.is_running():
                break
            if d.time - last_sync >= 0.01:
                viewer_handle.sync()
                last_sync = d.time
            # 尽量按实时速度播放
            lag = (time.time() - wall0) - d.time + RAMP_TIME
            if lag < 0:
                time.sleep(min(-lag, 0.005))

    if viewer_handle is not None:
        viewer_handle.close()

    rows = sim.rows
    if not rows:
        print("FAIL  没有采到数据")
        return False

    if csv_path is None:
        csv_path = os.path.join(LOG_DIR, f"control_sim_{mode}.csv")
    save_csv(csv_path, rows)
    print(f"  INFO  记录 {len(rows)} 步 -> {csv_path}")

    ok = summarize(sim, rows, mode, RAMP_TIME + t_run)

    if plot:
        plot_result(np.asarray(rows), mode, sim,
                    os.path.splitext(csv_path)[0] + ".png")
    return ok


def plot_result(rows, mode, sim, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = rows[:, 0]
    q = rows[:, 1:7]
    qdes = rows[:, 13:19]
    f = rows[:, 25:31]
    res = rows[:, 31:34]

    fig, axes = plt.subplots(3, 2, figsize=(11, 8.5), sharex=True)
    for i, ax in enumerate(axes[:, 0]):
        for j in range(2):
            k = i * 2 + j
            ax.plot(t, q[:, k], label="actual")
            ax.plot(t, qdes[:, k], "--", lw=1, label="target")
        ax.set_ylabel("q")
        ax.legend(fontsize=7, loc="best")
    for i, ax in enumerate(axes[:, 1]):
        for j in range(2):
            ax.plot(t, f[:, i * 2 + j], lw=0.8)
        ax.set_ylabel("force")
    axes[-1, 0].set_xlabel("t (s)")
    axes[-1, 1].set_xlabel("t (s)")
    fig.suptitle(f"control_sim {mode}")
    fig.tight_layout()

    fig2, ax2 = plt.subplots(figsize=(6, 4))
    ax2.plot(t, res)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("loop residual (mm)")
    ax2.set_title("equality constraint residual")
    fig2.tight_layout()

    fig.savefig(png, dpi=130)
    fig2.savefig(png.replace(".png", "_residual.png"), dpi=130)
    print(f"  INFO  图 -> {png}")
    plt.close(fig)
    plt.close(fig2)


def main():
    ap = argparse.ArgumentParser(description="linear_arm 1 kHz 运控仿真")
    ap.add_argument("--mode", choices=["step", "sine", "cartesian", "all"],
                    default="step")
    ap.add_argument("--duration", type=float, default=None,
                    help="总时长 (s)，含 1 s 起步过渡")
    ap.add_argument("--viewer", action="store_true", help="打开 MuJoCo 实时视图")
    ap.add_argument("--csv", default=None, help="CSV 输出路径")
    ap.add_argument("--plot", action="store_true", help="生成 matplotlib 曲线图")
    args = ap.parse_args()

    modes = ["step", "sine", "cartesian"] if args.mode == "all" else [args.mode]
    default_dur = {"step": 8.5, "sine": 9.0, "cartesian": 11.0}
    results = {}

    print(f"MuJoCo {mujoco.__version__}  MJCF {MJCF}")
    for mode in modes:
        dur = args.duration or default_dur[mode]
        csv = args.csv
        if csv and len(modes) > 1:
            csv = os.path.splitext(csv)[0] + f"_{mode}.csv"
        results[mode] = run_mode(mode, dur, args.viewer, csv, args.plot)

    print("\n" + "=" * 60)
    bad = [k for k, v in results.items() if not v]
    if bad:
        print(f"总结论: 有 {len(bad)} 项 FAIL -> {bad}")
    else:
        print("总结论: 全部通过")
    print("=" * 60)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
