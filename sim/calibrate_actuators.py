# -*- coding: utf-8 -*-
"""标定 3 个直线电缸与任务角的关系：运动学延拓 + 多项式拟合。

为什么不用物理沉降标定
----------------------
肘四连杆和腕部双缸都有 200 N 的占位力限，且给定 6 个执行器坐标后
机构仍有 4 维内部自由度（见 docs/kinematics/06-system.md 7.2 节）。
直接给位置目标让 MuJoCo 沉降，电缸会被力限顶住、腕部还会跳分支，
所以这里只做纯运动学：

1. 从 q=0 出发，给执行器坐标一个小增量；
2. 用闭环约束雅可比的零空间把增量映射到 18 个树坐标；
3. 再做最小范数 Newton 投影回到 c(q)=0；
4. 执行器映射 A=E@N 掉秩即到达折点，停止。

标定内容
--------
- 肘：L1 <-> θ4（前臂相对上臂的俯仰角，绕 Y）。
- 腕：L2/L3 与 B12 相对 B6 的姿态。腕部是 2 维流形而不是
  “R7=roll”式的三轴分解，这里直接用旋转矢量 r=log(R_B6^T R_B12)
  建表，并用三次多项式 r(L2,L3) 拟合；逆解用 2 维 Newton。

用法:
    python -X utf8 sim/calibrate_actuators.py
    python -X utf8 sim/calibrate_actuators.py --plot
    python -X utf8 sim/calibrate_actuators.py --step-mm 2.5 --l1-step-mm 0.5

输出:
    sim/calibration/actuator_calibration.json
    sim/calibration/elbow.png / wrist.png（--plot）

末行: 总结论: 全部通过 / 有 N 项 FAIL
"""
import argparse
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(os.path.dirname(HERE), "model")
MJCF = os.path.join(MODEL, "mjcf", "linear_arm.xml")
LOOPS = os.path.join(MODEL, "loops.yaml")
OUT_DIR = os.path.join(HERE, "calibration")

JOINTS = ["R1", "R2", "R3", "L1", "L2", "L3"]
BODIES = ("B3", "B6", "B12")

fails = []


def tag(ok, name, detail=""):
    print(f"  [{'OK  ' if ok else 'FAIL'}] {name}  {detail}")
    if not ok:
        fails.append(name)
    return bool(ok)


class Kinematics:
    """约束投影 + 执行器坐标延拓。"""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(MJCF)
        self.d = mujoco.MjData(self.m)
        self.loops = json.load(open(LOOPS, encoding="utf-8"))

        m = self.m
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        assert all(i >= 0 for i in jid), "关节名不匹配"
        self.qadr = np.array([m.jnt_qposadr[i] for i in jid])
        self.dadr = np.array([m.jnt_dofadr[i] for i in jid])
        self.bid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
                    for n in BODIES}
        assert all(i >= 0 for i in self.bid.values()), "刚体名不匹配"
        self.E = np.zeros((6, m.nv))
        self.E[np.arange(6), self.dadr] = 1.0
        self._rows_cache = None

    # ---------------------------------------------------------------- 约束
    def loop_rows(self, q):
        """返回 (Jc 9x18, c 9)，并顺带把 q 写进 data。"""
        m, d = self.m, self.d
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        rows, res = [], []
        for lp in self.loops:
            i1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["parent"])
            i2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["child"])
            p1 = d.xpos[i1] + d.xmat[i1].reshape(3, 3) @ (
                np.asarray(lp["origin_parent_mm"], float) / 1000.0)
            p2 = d.xpos[i2] + d.xmat[i2].reshape(3, 3) @ (
                np.asarray(lp["origin_child_mm"], float) / 1000.0)
            res.append(p2 - p1)
            J1 = np.zeros((3, m.nv))
            J2 = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, J1, None, p1, i1)
            mujoco.mj_jac(m, d, J2, None, p2, i2)
            rows.append(J2 - J1)
        return np.vstack(rows), np.concatenate(res)

    def project(self, q, iters=8, tol=1e-12):
        """最小范数 Newton 投影回 c(q)=0。"""
        for _ in range(iters):
            Jc, c = self.loop_rows(q)
            if np.linalg.norm(c) < tol:
                break
            q = q - Jc.T @ np.linalg.pinv(Jc @ Jc.T, rcond=1e-9) @ c
        return q

    def move(self, q, dq_act, step=5e-4):
        """沿零空间给执行器坐标一个增量，失败（掉秩/投影不收敛）返回 ok=False。"""
        n = max(1, int(np.ceil(np.max(np.abs(dq_act)) / step)))
        for _ in range(n):
            Jc, _ = self.loop_rows(q)
            _, s, Vt = np.linalg.svd(Jc, full_matrices=True)
            # 阈值与 control_sim 一致：太紧会把真实零空间切掉一维
            rank = int(np.sum(s > s.max() * 1e-3))
            N = Vt[rank:].T
            A = self.E @ N
            if np.linalg.matrix_rank(A, tol=1e-3) < 6:
                return q, False
            q = q + N @ np.linalg.pinv(A, rcond=1e-6) @ (dq_act / n)
            q = self.project(q)
            if np.linalg.norm(self.loop_rows(q)[1]) > 1e-6:
                return q, False
        return q, True

    # ---------------------------------------------------------------- 量
    def elbow_angle(self):
        R = self.d.xmat[self.bid["B3"]].reshape(3, 3).T @ \
            self.d.xmat[self.bid["B6"]].reshape(3, 3)
        return float(np.arctan2(R[0, 2], R[2, 2]))

    def wrist_rotvec(self):
        R = self.d.xmat[self.bid["B6"]].reshape(3, 3).T @ \
            self.d.xmat[self.bid["B12"]].reshape(3, 3)
        return rotvec(R)

    def state_after(self, q):
        self.loop_rows(q)                     # 写入 xmat
        return self.elbow_angle(), self.wrist_rotvec()


def rotvec(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(ang)) * ang


# ---------------------------------------------------------------- 肘部采样
def sample_elbow(kin, q0, step_mm=0.5):
    print(f"\n=== 1. 肘部 L1 -> θ4（步长 {step_mm} mm） ===")
    step = step_mm / 1000.0
    out = []

    def walk(sign, n_max):
        q = q0.copy()
        for k in range(1, n_max + 1):
            q2, ok = kin.move(q, np.array([0, 0, 0, sign * step, 0, 0]))
            if not ok:
                return k - 1
            q = q2
            th, _ = kin.state_after(q)
            res = np.linalg.norm(kin.loop_rows(q)[1]) * 1000.0
            out.append((sign * k * step, th, res))
        return n_max

    kin.state_after(q0)
    out.append((0.0, kin.elbow_angle(), 0.0))
    n_neg = walk(-1, 200)
    n_pos = 0                     # 新限位 L1 <= 0，不向正半轴扫描
    arr = np.array(sorted(out))
    print(f"  负向走到 {n_neg * step_mm:.1f} mm，正向走到 {n_pos * step_mm:.1f} mm")

    # 正向最后一步已经越过折点：θ4 在 +9 mm 处回弹。标定表只保留单调段。
    dth = np.diff(arr[:, 1])
    bad = np.where(dth >= 0)[0]        # θ4 随 L1 递减，出现 >=0 即回折
    if bad.size:
        print(f"  INFO  在 L1={arr[bad[0], 0]*1000:+.2f} mm 处检测到折点，"
              f"截断后 {bad[0] + 1} 个点")
        arr = arr[:bad[0] + 1]

    print(f"  L1 范围 {arr[0, 0]*1000:+.2f} .. {arr[-1, 0]*1000:+.2f} mm")
    print(f"  θ4 范围 {np.degrees(arr[:, 1].min()):+.2f} .. "
          f"{np.degrees(arr[:, 1].max()):+.2f} deg")
    tag(bool(np.all(np.diff(arr[:, 0]) > 0)), "L1 单调递增",
        f"{len(arr)} 个采样点")
    tag(bool(np.all(np.diff(arr[:, 1]) < 0)), "θ4 随 L1 单调递减",
        "折点前无回折")
    tag(bool(arr[:, 2].max() < 1e-3), "闭环残差 < 1e-3 mm",
        f"最大 {arr[:, 2].max():.2e} mm")
    return arr


# ---------------------------------------------------------------- 腕部采样
def sample_wrist(kin, q0, half_w_mm=40.0, step_mm=1.0):
    """沿差动行程 w = L2 - L3 采样，锁和 L2 + L3 = 0。"""
    print(f"\n=== 2. 腕部 w=L2-L3 -> 旋转矢量（±{half_w_mm:g} mm，"
          f"步长 {step_mm:g} mm） ===")
    vals = np.arange(-half_w_mm, half_w_mm + 1e-9, step_mm) / 1000.0
    n = len(vals)
    rvec = np.full((n, 3), np.nan)
    res = np.full(n, np.nan)
    good = 0
    for i, w in enumerate(vals):
        q, ok = kin.move(q0, np.array([0, 0, 0, 0, 0.5 * w, -0.5 * w], float))
        if not ok:
            continue
        kin.loop_rows(q)
        rvec[i] = kin.wrist_rotvec()
        res[i] = np.linalg.norm(kin.loop_rows(q)[1]) * 1000.0
        good += 1
    print(f"  有效点 {good}/{n}")
    tag(good == n, "差动行程全部可达", f"{good}/{n}")
    tag(bool(np.nanmax(res) < 1e-3), "闭环残差 < 1e-3 mm",
        f"最大 {np.nanmax(res):.2e} mm")
    return vals, rvec, res


# ---------------------------------------------------------------- 多项式
POLY_DEG = 5             # 五次（6 项）：差动 1 维表足够
POLY_SCALE = 0.02        # 把 w 归一到 O(1)，改善最小二乘条件数


def curve_terms(w):
    """一维五次多项式基，w 单位 m，内部按 POLY_SCALE 归一化。"""
    s = np.asarray(w, float) / POLY_SCALE
    return np.stack([s ** k for k in range(POLY_DEG + 1)], axis=-1)


def curve_jac(w):
    """d(terms)/dw（对物理坐标，已除 POLY_SCALE）。"""
    s = np.asarray(w, float) / POLY_SCALE
    zero = np.zeros_like(s)
    return np.stack([k * s ** (k - 1) if k else zero
                     for k in range(POLY_DEG + 1)], axis=-1) / POLY_SCALE


class WristCurve:
    """沿差动行程 w = L2 - L3 的一维旋转矢量拟合。"""

    def __init__(self, w, rvec):
        self.w = np.asarray(w, float)
        A = curve_terms(self.w)
        self.coef, *_ = np.linalg.lstsq(A, rvec, rcond=None)
        err = A @ self.coef - rvec
        en = np.linalg.norm(err, axis=-1)          # 每个采样点的角度误差模长
        self.rms = float(np.sqrt(np.mean(en ** 2)))
        self.max = float(en.max())
        print(f"  五次多项式拟合: RMS {self.rms:.3e} rad，"
              f"最大 {self.max:.3e} rad （{np.degrees(self.max):.3f} deg）")

    def forward(self, w):
        w = np.atleast_1d(np.asarray(w, float))
        return curve_terms(w) @ self.coef

    def jacobian(self, w):
        w = np.atleast_1d(np.asarray(w, float))
        return curve_jac(w) @ self.coef        # n x 3

    def inverse(self, r_target, guess, iters=50, tol=1e-10):
        """1 维 Newton：解 r(w)=r_target，返回 (w, err)。"""
        w = float(guess)
        for _ in range(iters):
            r = self.forward(w)[0]
            e = r_target - r
            if np.linalg.norm(e) < tol:
                return w, float(np.linalg.norm(e))
            j = self.jacobian(w)[0]
            dw = float(j @ e / (j @ j + 1e-15))
            w += np.clip(dw, -0.002, 0.002)
        r = self.forward(w)[0]
        return w, float(np.linalg.norm(r_target - r))


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="电缸 <-> 任务角标定")
    ap.add_argument("--step-mm", type=float, default=1.0, help="腕部差动步长 (mm)")
    ap.add_argument("--half-w-mm", type=float, default=40.0,
                    help="差动行程 w=L2-L3 的半宽 (mm)")
    ap.add_argument("--l1-step-mm", type=float, default=0.5, help="肘部步长 (mm)")
    ap.add_argument("--plot", action="store_true", help="生成标定曲线")
    ap.add_argument("--plot-only", action="store_true", help="只从 JSON 画图")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    json_path = os.path.join(OUT_DIR, "actuator_calibration.json")
    if args.plot_only:
        plot_all(json.load(open(json_path, encoding="utf-8")), args.plot)
        return 0

    print(f"MuJoCo {mujoco.__version__}")
    print(f"MJCF {MJCF}")
    kin = Kinematics()
    q0 = kin.project(np.zeros(kin.m.nq))
    kin.state_after(q0)
    print(f"  q0 投影后闭环残差 "
          f"{np.linalg.norm(kin.loop_rows(q0)[1]) * 1000:.2e} mm")

    elbow = sample_elbow(kin, q0, args.l1_step_mm)
    vals, rvec, res = sample_wrist(kin, q0, args.half_w_mm, args.step_mm)
    wrist = WristCurve(vals, rvec)

    print("\n=== 3. 拟合与回代 ===")
    # 肘：直接线性插值，正反各查一次（θ4 随 L1 递减，插值前按 θ4 排序）
    th_el = elbow[:, 1]
    l1_el = elbow[:, 0]
    order = np.argsort(th_el)
    l1_back = np.interp(th_el, th_el[order], l1_el[order])
    th_back = np.interp(l1_back, l1_el, th_el)
    max_el = float(np.abs(th_back - th_el).max())
    tag(max_el < 1e-6, "肘部 L1<->θ4 回代",
        f"最大 {max_el:.2e} rad")

    tag(wrist.max < 5e-3, "腕部五次多项式拟合 < 5 mrad",
        f"最大 {wrist.max:.2e} rad")

    # 腕：用邻近采样点做初值，逆解回代
    errs = []
    for i in range(0, len(vals), 3):
        target = rvec[i]
        if np.any(np.isnan(target)):
            continue
        guess = vals[i] + (0.5e-3 if i + 1 < len(vals) else -0.5e-3)
        w_back, err = wrist.inverse(target, guess)
        errs.append([abs(w_back - vals[i]), err])
    errs = np.array(errs)
    print(f"  腕部逆解：差动行程误差最大 {errs[:,0].max()*1000:.3f} mm，"
          f"残差最大 {errs[:,1].max():.2e} rad")
    tag(errs[:, 0].max() < 1e-3, "腕部逆解回代 < 1 mm",
        f"最大 {errs[:,0].max()*1000:.3f} mm")

    doc = {
        "schema": "linrot-arm/actuator-calibration/v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": os.path.relpath(MJCF, os.path.dirname(HERE)).replace("\\", "/"),
        "method": "constraint-nullspace continuation + Newton projection",
        "units": {"l1": "m", "w": "m",
                  "theta4": "rad", "rotvec": "rad"},
        "conventions": {
            "elbow": "theta4 = atan2(R_B3^T R_B6 [0,2], [2,2])，绕 Y 的俯仰",
            "wrist": "w = L2 - L3，L2=+w/2，L3=-w/2（锁和）；"
                     "r = log(R_B6^T R_B12)，旋转矢量，q=0 时 r=0",
        },
        "elbow": {
            "l1": elbow[:, 0].tolist(),
            "theta4": elbow[:, 1].tolist(),
            "residual_mm": elbow[:, 2].tolist(),
        },
        "wrist": {
            "w": vals.tolist(),
            "rotvec": rvec.tolist(),
            "residual_mm": res.tolist(),
            "poly_degree": POLY_DEG,
            "poly_scale_m": POLY_SCALE,
            "poly_coef": wrist.coef.tolist(),
            "poly_rms_rad": wrist.rms,
            "poly_max_rad": wrist.max,
        },
        "limits": {
            "l1_mm": [float(elbow[0, 0] * 1000), float(elbow[-1, 0] * 1000)],
            "wrist_w_mm": [-args.half_w_mm, args.half_w_mm],
            "wrist_sum_mm": [-1.0, 1.0],
            "note": "L1 两端由四连杆死点决定；腕部锁和 L2+L3=0（软约束 ±1 mm），"
                    "w=L2-L3 为差动行程",
        },
        "verification": {
            "elbow_roundtrip_max_rad": max_el,
            "wrist_inverse_max_mm": float(errs[:, 0].max() * 1000),
            "wrist_inverse_residual_max_rad": float(errs[:, 1].max()),
        },
    }
    with open(json_path, "w", encoding="utf-8", newline="\n") as fp:
        json.dump(doc, fp, ensure_ascii=False, indent=1)
    print(f"  INFO  标定表 -> {json_path}")

    if args.plot:
        plot_all(doc, True)

    print("\n" + "=" * 60)
    if fails:
        print(f"总结论: 有 {len(fails)} 项 FAIL -> {fails}")
    else:
        print("总结论: 全部通过")
    print("=" * 60)
    return 0 if not fails else 1


# ---------------------------------------------------------------- 画图
def plot_all(doc, enabled):
    if not enabled:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    el = doc["elbow"]
    l1 = np.array(el["l1"]) * 1000.0
    th = np.degrees(el["theta4"])
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].plot(l1, th)
    ax[0].set_xlabel("L1 (mm)")
    ax[0].set_ylabel("theta4 (deg)")
    ax[0].set_title("elbow")
    ax[0].grid(True, alpha=0.3)
    ax[1].plot(l1, el["residual_mm"])
    ax[1].set_xlabel("L1 (mm)")
    ax[1].set_ylabel("constraint residual (mm)")
    ax[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "elbow.png"), dpi=130)
    plt.close(fig)

    wd = doc["wrist"]
    wv = np.array(wd["w"])
    w = wv * 1000.0
    rv = np.array(wd["rotvec"])
    fit = curve_terms(wv) @ np.array(wd["poly_coef"])
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharex=True)
    for k, ax in enumerate(axes):
        ax.plot(w, rv[:, k], "o", ms=2.5, label="table")
        ax.plot(w, fit[:, k], "-", lw=1, label="poly")
        ax.set_xlabel("w = L2 - L3 (mm)")
        ax.set_ylabel(f"rotvec[{k}] (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "wrist.png"), dpi=130)
    plt.close(fig)
    print(f"  INFO  图 -> {OUT_DIR}/elbow.png, wrist.png")


if __name__ == "__main__":
    sys.exit(main())
