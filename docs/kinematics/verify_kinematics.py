#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线旋转混合驱动机械臂：递推运动学数值校核。

本脚本只做运动学正解、雅可比、加速度和闭环约束的数值校核，
不包含任何控制算法、逆解迭代或电机驱动逻辑。

用法：
    python docs/kinematics/verify_kinematics.py
    python docs/kinematics/verify_kinematics.py --table

约定：
    - 18 个运动体，编号 1..18，body i 由 joint i 驱动；
    - q, qd, qdd 均为 18 维，顺序与 MuJoCo 的 joint 顺序一致；
    - 所有 6 维速度/加速度均为 [线分量; 角分量]，且投影在对应体坐标系；
    - 闭环约束采用 3 行/环的球铰（connect）模型。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
MJCF = os.path.join(ROOT, "model", "mjcf", "linear_arm.xml")
LOOPS = os.path.join(ROOT, "model", "loops.yaml")

ACTUATED = {"R1", "R2", "R3", "L1", "L2", "L3"}


def skew(v):
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rodrigues(n, q):
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    K = skew(n)
    return np.eye(3) * np.cos(q) + np.sin(q) * K + (1.0 - np.cos(q)) * np.outer(n, n)


def frame6(A, v):
    """把体坐标 6 维向量变换到世界系。"""
    return np.r_[A @ v[:3], A @ v[3:]]


class Tree:
    """从 MJCF 提取 18 个运动体的派生树。

    B0 与 world 固结，B10/B11 与各自父体固结；这三者运动学上并入父体，
    因此 22 个 MJCF body 化为 1 个 ground + 18 个运动体。
    """

    def __init__(self, mjcf_path=MJCF):
        self.names = ["world"]
        self.parent = [-1]
        self.joint = [None]
        self.type = [None]
        self.o = [np.zeros(3)]
        self.n = [np.zeros(3)]
        self.alias = {"world": 0}
        root = ET.parse(mjcf_path).getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise RuntimeError("MJCF 中找不到 worldbody")
        self._walk(worldbody, 0)

    def _walk(self, elem, parent):
        for body in elem.findall("body"):
            name = body.get("name")
            pos = np.array([float(x) for x in body.get("pos", "0 0 0").split()])
            if "quat" in body.attrib or "euler" in body.attrib or "axisangle" in body.attrib:
                raise RuntimeError(f"body {name} 使用了非零初始姿态，脚本未实现")
            joint = body.find("joint")
            if name == "B0":
                if np.linalg.norm(pos) > 1e-15:
                    raise RuntimeError("B0 不在 world 原点")
                self.alias[name] = parent
                self._walk(body, parent)
            elif joint is None:
                if np.linalg.norm(pos) > 1e-15:
                    raise RuntimeError(f"固结体 {name} 相对父体有非零偏移")
                self.alias[name] = parent
                self._walk(body, parent)
            else:
                cls = joint.get("class", "")
                jtype = "P" if (joint.get("type") == "slide" or cls == "slide") else "R"
                axis = np.array([float(x) for x in joint.get("axis", "0 0 1").split()])
                nax = np.linalg.norm(axis)
                if nax < 1e-15:
                    raise RuntimeError(f"joint {name} 轴长为零")
                axis = axis / nax
                jpos = np.array([float(x) for x in joint.get("pos", "0 0 0").split()])
                if np.linalg.norm(jpos) > 1e-15:
                    raise RuntimeError(f"joint {name} 的 pos 非零，脚本未实现")
                idx = len(self.names)
                self.names.append(name)
                self.parent.append(parent)
                self.joint.append(joint.get("name"))
                self.type.append(jtype)
                self.o.append(pos)
                self.n.append(axis)
                self.alias[name] = idx
                self._walk(body, idx)

    @property
    def nbody(self):
        return len(self.names) - 1

    def ancestor_sets(self):
        pvec = [set() for _ in range(self.nbody + 1)]
        for i in range(1, self.nbody + 1):
            pvec[i] = pvec[self.parent[i]] | {i}
        return pvec

    def lca(self, b, m):
        """两个体的最低公共祖先（含自身）。"""
        anc = set()
        x = b
        while x >= 0:
            anc.add(x)
            x = self.parent[x]
        x = m
        while x not in anc:
            x = self.parent[x]
        return x


def parse_loops(tree: Tree, path=LOOPS):
    # 文件名是 loops.yaml，内容实际为 JSON 数组。
    raw = json.load(open(path, encoding="utf-8"))
    loops = []
    for item in raw:
        loops.append(
            dict(
                name=item["joint"],
                b=tree.alias[item["parent"]],
                m=tree.alias[item["child"]],
                p1=np.array(item["origin_parent_mm"], dtype=float) / 1000.0,
                p2=np.array(item["origin_child_mm"], dtype=float) / 1000.0,
                axis=np.array(item["axis"], dtype=float)
                / np.linalg.norm(item["axis"]),
            )
        )
    return loops


class Kinematics:
    def __init__(self, tree: Tree):
        self.t = tree
        self.N = tree.nbody

    # ---------- 位置级 ----------
    def fk(self, q):
        t = self.t
        r = np.zeros((self.N + 1, 3))
        A = np.zeros((self.N + 1, 3, 3))
        A[0] = np.eye(3)
        for i in range(1, self.N + 1):
            p = t.parent[i]
            if t.type[i] == "R":
                Api = rodrigues(t.n[i], q[i - 1])
                s = np.zeros(3)
            else:
                Api = np.eye(3)
                s = t.n[i] * q[i - 1]
            r[i] = r[p] + A[p] @ (t.o[i] + s)
            A[i] = A[p] @ Api
        return r, A

    # ---------- 速度级 ----------
    def velocity(self, q, qd):
        """返回 r, A, v, J, b。

        v[i] = v_{0,i|i}，6 维，投影在 i 系；
        J[i] = ∂v_{0,i|i}/∂qd，6×18；
        b[i] = qdd=0 时的偏置加速度 a_{0,i|i}。
        """
        t = self.t
        r, A = self.fk(q)
        v = np.zeros((self.N + 1, 6))
        J = np.zeros((self.N + 1, 6, self.N))
        b = np.zeros((self.N + 1, 6))
        for i in range(1, self.N + 1):
            p = t.parent[i]
            Api = A[p].T @ A[i]
            if t.type[i] == "R":
                rho = t.o[i]
                rdot = np.zeros(3)
                om_rel = t.n[i] * qd[i - 1]
                Phi = np.r_[np.zeros(3), t.n[i]]
            else:
                rho = t.o[i] + t.n[i] * q[i - 1]
                rdot = t.n[i] * qd[i - 1]
                om_rel = np.zeros(3)
                Phi = np.r_[t.n[i], np.zeros(3)]
            C = np.zeros((6, 6))
            C[:3, :3] = Api.T
            C[:3, 3:] = Api.T @ skew(rho).T
            C[3:, 3:] = Api.T
            v[i] = C @ v[p] + Phi * qd[i - 1]
            J[i] = C @ J[p]
            J[i][:, i - 1] = Phi
            wp = v[p][3:]
            eta = np.r_[
                Api.T @ (skew(wp) @ skew(wp) @ rho + 2.0 * skew(wp) @ rdot),
                Api.T @ (skew(wp) @ om_rel),
            ]
            b[i] = C @ b[p] + eta
        return r, A, v, J, b

    # ---------- 加速度级 ----------
    def acceleration(self, q, qd, qdd):
        t = self.t
        r, A = self.fk(q)
        v = np.zeros((self.N + 1, 6))
        a = np.zeros((self.N + 1, 6))
        for i in range(1, self.N + 1):
            p = t.parent[i]
            Api = A[p].T @ A[i]
            if t.type[i] == "R":
                rho = t.o[i]
                rdot = np.zeros(3)
                om_rel = t.n[i] * qd[i - 1]
                Phi = np.r_[np.zeros(3), t.n[i]]
            else:
                rho = t.o[i] + t.n[i] * q[i - 1]
                rdot = t.n[i] * qd[i - 1]
                om_rel = np.zeros(3)
                Phi = np.r_[t.n[i], np.zeros(3)]
            C = np.zeros((6, 6))
            C[:3, :3] = Api.T
            C[:3, 3:] = Api.T @ skew(rho).T
            C[3:, 3:] = Api.T
            v[i] = C @ v[p] + Phi * qd[i - 1]
            wp = v[p][3:]
            eta = np.r_[
                Api.T @ (skew(wp) @ skew(wp) @ rho + 2.0 * skew(wp) @ rdot),
                Api.T @ (skew(wp) @ om_rel),
            ]
            a[i] = C @ a[p] + Phi * qdd[i - 1] + eta
        return r, A, v, a

    # ---------- 任意祖先的牵连矩阵 ----------
    def ancestor_C(self, r, A, i, k):
        """式 (9.51)：C_{i,k}，k ∈ pvec[i]。"""
        Aki = A[k].T @ A[i]
        r_ki = A[k].T @ (r[i] - r[k])
        C = np.zeros((6, 6))
        C[:3, :3] = Aki.T
        C[:3, 3:] = Aki.T @ skew(r_ki).T
        C[3:, 3:] = Aki.T
        return C

    def relative_C(self, r, A, i):
        p = self.t.parent[i]
        Api = A[p].T @ A[i]
        rho = A[p].T @ (r[i] - r[p])
        C = np.zeros((6, 6))
        C[:3, :3] = Api.T
        C[:3, 3:] = Api.T @ skew(rho).T
        C[3:, 3:] = Api.T
        return C

    def product_C(self, r, A, i, k):
        """式 (9.54) 的连乘形式。"""
        path = []
        x = i
        while x != k:
            if x < 0:
                raise ValueError("k 不是 i 的祖先")
            path.append(x)
            x = self.t.parent[x]
        C = np.eye(6)
        for x in reversed(path):
            C = self.relative_C(r, A, x) @ C
        return C

    # ---------- 点与闭环 ----------
    @staticmethod
    def point_transform(p):
        T = np.zeros((6, 6))
        T[:3, :3] = np.eye(3)
        T[:3, 3:] = -skew(p)
        T[3:, 3:] = np.eye(3)
        return T

    def point_terms(self, J, b, v, i, p):
        """体 i 上局部点 p 的 6×18 雅可比与偏置加速度（体坐标）。"""
        T = self.point_transform(p)
        w = v[i][3:]
        bp = T @ b[i] + np.r_[skew(w) @ skew(w) @ p, np.zeros(3)]
        return T @ J[i], bp

    def loop_terms(self, r, A, J, v, b, loops):
        """返回 c (9×1), Jc (9×18), bc (9×1)，均为世界系。

        采用讲义定义 c = r_M - r_B（动点减基点）。
        """
        cs, Jcs, bcs = [], [], []
        for lp in loops:
            i, j = lp["b"], lp["m"]
            p1, p2 = lp["p1"], lp["p2"]
            cs.append(r[j] + A[j] @ p2 - (r[i] + A[i] @ p1))
            J1, b1 = self.point_terms(J, b, v, i, p1)
            J2, b2 = self.point_terms(J, b, v, j, p2)
            Jcs.append(A[j] @ J2[:3] - A[i] @ J1[:3])
            bcs.append(A[j] @ b2[:3] - A[i] @ b1[:3])
        return np.vstack(cs), np.vstack(Jcs), np.vstack(bcs)

    def task_terms(self, r, A, v, J, b, body=13, p=np.zeros(3)):
        """任务点在世界系的 6×18 雅可比与偏置加速度。"""
        Jp, bp = self.point_terms(J, b, v, body, p)
        Ab = A[body]
        Jw = np.vstack([Ab @ Jp[:3], Ab @ Jp[3:]])
        bw = np.r_[Ab @ bp[:3], Ab @ bp[3:]]
        return Jw, bw


def random_state(tree: Tree, rng):
    """按关节类型生成在限位内的随机状态。"""
    q = np.zeros(tree.nbody)
    qd = np.zeros(tree.nbody)
    qdd = np.zeros(tree.nbody)
    for i in range(1, tree.nbody + 1):
        name = tree.joint[i]
        if tree.type[i] == "P":
            q[i - 1] = rng.uniform(-0.05, 0.05)
            qd[i - 1] = rng.uniform(-0.05, 0.05)
            qdd[i - 1] = rng.uniform(-0.05, 0.05)
        elif name.startswith("L2g") or name.startswith("L3g"):
            q[i - 1] = rng.uniform(-0.3, 0.3)
            qd[i - 1] = rng.uniform(-0.2, 0.2)
            qdd[i - 1] = rng.uniform(-0.2, 0.2)
        else:
            q[i - 1] = rng.uniform(-0.5, 0.5)
            qd[i - 1] = rng.uniform(-0.3, 0.3)
            qdd[i - 1] = rng.uniform(-0.2, 0.2)
    return q, qd, qdd


def fmt_vec(v, nd=6):
    return "[" + ", ".join(f"{x:.{nd}f}" for x in v) + "]"


def print_table(tree, kin, q0):
    r0, A0 = kin.fk(q0)
    print("\nq=0 位形（世界系，单位 m）：")
    print("  i  body          joint       type  parent        r_i")
    for i in range(1, tree.nbody + 1):
        print(
            f"{i:3d}  {tree.names[i]:12s}  {str(tree.joint[i]):10s}  "
            f"{tree.type[i]:2s}  {tree.parent[i]:6d}  {fmt_vec(r0[i])}"
        )
    print("\nq=0 闭环锚点（世界系，单位 m）：")
    for lp in parse_loops(tree):
        b, m = lp["b"], lp["m"]
        print(
            f"  {lp['name']:4s}  B=({fmt_vec(r0[b] + A0[b] @ lp['p1'])})"
            f"  M=({fmt_vec(r0[m] + A0[m] @ lp['p2'])})"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description="linrot-arm 递推运动学数值校核")
    parser.add_argument("--seed", type=int, default=20261122)
    parser.add_argument("--table", action="store_true", help="打印 q=0 数值表")
    parser.add_argument("--quick", action="store_true", help="只跑一组随机状态")
    args = parser.parse_args(argv)

    tree = Tree()
    kin = Kinematics(tree)
    loops = parse_loops(tree)
    N = tree.nbody
    q0 = np.zeros(N)

    print("=" * 78)
    print("linrot-arm 递推运动学数值校核（只校核，不是控制器）")
    print("=" * 78)
    print(f"MJCF       : {os.path.relpath(MJCF, ROOT)}")
    print(f"loops      : {os.path.relpath(LOOPS, ROOT)}")
    print(f"运动体数   : {N}")
    print(f"闭环数     : {len(loops)}（每环 3 行 connect）")
    print(f"执行器关节 : {', '.join(tree.joint[i] for i in range(1, N + 1) if tree.joint[i] in ACTUATED)}")

    if args.table:
        print_table(tree, kin, q0)

    # ---- q=0 闭环与秩 ----
    r0, A0, v0, J0, b0 = kin.velocity(q0, np.zeros(N))
    c0, Jc0, bc0 = kin.loop_terms(r0, A0, J0, v0, b0, loops)
    print("\n[1] q=0 闭环残差")
    for lp, c in zip(loops, c0):
        print(f"  {lp['name']:4s}  |c| = {np.linalg.norm(c) * 1e3:.6f} mm")

    s_c = np.linalg.svd(Jc0, compute_uv=False)
    rank_c = int(np.sum(s_c > 1e-6))
    U, _, Vt = np.linalg.svd(Jc0, full_matrices=True)
    null_dim = N - rank_c
    Nsp = Vt[rank_c:].T
    print("\n[2] q=0 闭环雅可比 Jc (9×18)")
    print(f"  奇异值 : {np.array2string(s_c, precision=6)}")
    print(f"  秩     : {rank_c}，零空间维数 : {null_dim}")
    print(f"  R6 块秩 : {np.linalg.matrix_rank(Jc0[0:3], tol=1e-6)}"
          f"（平面四连杆，Y 行为结构冗余）")
    print(f"  R10 块秩: {np.linalg.matrix_rank(Jc0[3:6], tol=1e-6)}")
    print(f"  R12 块秩: {np.linalg.matrix_rank(Jc0[6:9], tol=1e-6)}")

    Jtask0, _ = kin.task_terms(r0, A0, v0, J0, b0)
    s_t = np.linalg.svd(Jtask0, compute_uv=False)
    s_tn = np.linalg.svd(Jtask0 @ Nsp, compute_uv=False)
    print("\n[3] q=0 任务雅可比")
    print(f"  B12 原点 6×18 奇异值 : {np.array2string(s_t, precision=6)}")
    print(f"  秩 : {int(np.sum(s_t > 1e-6))}")
    print(f"  约束零空间上的 6×10 奇异值 : {np.array2string(s_tn, precision=6)}")
    print(f"  秩 : {int(np.sum(s_tn > 1e-6))}，内部自由度 : "
          f"{Nsp.shape[1] - int(np.sum(s_tn > 1e-6))}")

    # ---- 随机状态 ----
    rng = np.random.default_rng(args.seed)
    q, qd, qdd = random_state(tree, rng)
    r, A, v, J, bias = kin.velocity(q, qd)
    _, _, _, a = kin.acceleration(q, qd, qdd)
    c, Jc, bc = kin.loop_terms(r, A, J, v, bias, loops)
    Jtask, btask = kin.task_terms(r, A, v, J, bias)

    print("\n[4] 随机位形校核")
    print(f"  q   = {np.array2string(q, precision=6)}")
    print(f"  qd  = {np.array2string(qd, precision=6)}")
    print(f"  qdd = {np.array2string(qdd, precision=6)}")

    err = {}
    # 4.1 任意祖先 C：直接式、连乘式、J 的列
    err_C_direct = 0.0
    err_C_product = 0.0
    err_C_column = 0.0
    pvec = tree.ancestor_sets()
    for i in range(1, N + 1):
        for k in pvec[i]:
            Cd = kin.ancestor_C(r, A, i, k)
            Cp = kin.product_C(r, A, i, k)
            phi_k = J[k][:, k - 1]
            err_C_direct = max(err_C_direct, np.max(np.abs(Cd - Cp)))
            err_C_product = max(err_C_product, np.max(np.abs(Cp @ phi_k - J[i][:, k - 1])))
            err_C_column = max(err_C_column, np.max(np.abs(Cd @ phi_k - J[i][:, k - 1])))
    print(f"  C_{i,k} 直接式 vs 连乘式      : {err_C_direct:.3e}")
    print(f"  C_{i,k}Φ_k vs J_i 第 k 列     : {err_C_column:.3e}")

    # 4.2 闭环 c 的有限差分雅可比
    hJ = 1e-6
    Jc_fd = np.zeros_like(Jc)
    for k in range(N):
        dq = np.zeros(N)
        dq[k] = hJ
        rp, Ap = kin.fk(q + dq)
        rm, Am = kin.fk(q - dq)
        cp = np.vstack([rp[lp["m"]] + Ap[lp["m"]] @ lp["p2"]
                        - (rp[lp["b"]] + Ap[lp["b"]] @ lp["p1"]) for lp in loops])
        cm = np.vstack([rm[lp["m"]] + Am[lp["m"]] @ lp["p2"]
                        - (rm[lp["b"]] + Am[lp["b"]] @ lp["p1"]) for lp in loops])
        Jc_fd[:, k] = (cp - cm).reshape(-1) / (2.0 * hJ)
    err_Jc = np.max(np.abs(Jc - Jc_fd))
    print(f"  Jc 解析 vs c(q) 中心差分       : {err_Jc:.3e}")

    # 4.3 闭环速度的 Psi 形式
    # 讲义中的 v_{f,m|m} 是“在 f 系中的相对速度”，c_dot|f 也是 f 系坐标导数；
    # 因此用 c_f(q)=A_f^T c(q) 的中心差分作对照，避免把世界系导数与 f 系导数混淆。
    err_psi = 0.0
    hpsi = 1e-6
    for j, lp in enumerate(loops):
        b, m = lp["b"], lp["m"]
        f = tree.lca(b, m)
        Cmf = kin.ancestor_C(r, A, m, f)
        Cbf = kin.ancestor_C(r, A, b, f)
        v_rel_m = v[m] - Cmf @ v[f]
        v_rel_b = v[b] - Cbf @ v[f]
        Afm = A[f].T @ A[m]
        Afb = A[f].T @ A[b]
        Psi_m = np.hstack([Afm, Afm @ skew(lp["p2"]).T])
        Psi_b = np.hstack([Afb, Afb @ skew(lp["p1"]).T])
        cdot_psi = Psi_m @ v_rel_m - Psi_b @ v_rel_b

        def c_in_f(qq):
            rr, AA = kin.fk(qq)
            cw = rr[m] + AA[m] @ lp["p2"] - (rr[b] + AA[b] @ lp["p1"])
            return AA[f].T @ cw

        cfp = c_in_f(q + hpsi * qd)
        cfm = c_in_f(q - hpsi * qd)
        cdot_f_fd = (cfp - cfm) / (2.0 * hpsi)
        err_psi = max(err_psi, np.max(np.abs(cdot_psi - cdot_f_fd)))
    print(f"  Psi 形式 vs c_f 中心差分          : {err_psi:.3e}")

    # 4.4 加速度：世界速度的中心差分
    h = 1e-4
    qp = q + h * qd + 0.5 * h * h * qdd
    qm = q - h * qd + 0.5 * h * h * qdd
    _, Ap, vp, _, _ = kin.velocity(qp, qd + h * qdd)
    _, Am, vm, _, _ = kin.velocity(qm, qd - h * qdd)
    err_a = 0.0
    for i in range(1, N + 1):
        wp = frame6(Ap[i], vp[i])
        wm = frame6(Am[i], vm[i])
        fd = (wp - wm) / (2.0 * h)
        ana = frame6(A[i], a[i])
        err_a = max(err_a, np.max(np.abs(fd - ana)))
    print(f"  体加速度 vs 世界速度差分       : {err_a:.3e}")

    # 4.5 闭环 c_ddot
    rp, Ap, vp, Jp, bp = kin.velocity(qp, qd + h * qdd)
    rm, Am, vm, Jm, bm = kin.velocity(qm, qd - h * qdd)
    cp, _, _ = kin.loop_terms(rp, Ap, Jp, vp, bp, loops)
    cm, _, _ = kin.loop_terms(rm, Am, Jm, vm, bm, loops)
    cdd_fd = (cp - 2.0 * c + cm) / (h * h)
    cdd_ana = Jc @ qdd + bc.reshape(-1)
    err_cdd = np.max(np.abs(cdd_fd.reshape(-1) - cdd_ana))
    print(f"  闭环 c_ddot vs 位置二阶差分    : {err_cdd:.3e}")

    # 4.6 MuJoCo 对照
    err_r = err_A = err_J = err_v = err_Jtask = np.nan
    if mujoco is not None:
        model = mujoco.MjModel.from_xml_path(MJCF)
        data = mujoco.MjData(model)
        if model.nv != N:
            print(f"  FAIL MuJoCo nv={model.nv} != {N}")
        data.qpos[:] = q
        data.qvel[:] = qd
        mujoco.mj_forward(model, data)
        err_r = 0.0
        err_A = 0.0
        err_J = 0.0
        err_v = 0.0
        for i in range(1, N + 1):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, tree.names[i])
            err_r = max(err_r, np.linalg.norm(r[i] - data.xpos[bid]))
            err_A = max(err_A, np.max(np.abs(A[i] - data.xmat[bid].reshape(3, 3))))
            Jp = np.zeros((3, model.nv))
            Jr = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, Jp, Jr, data.xpos[bid], bid)
            err_J = max(
                err_J,
                np.max(np.abs(A[i] @ J[i][:3] - Jp)),
                np.max(np.abs(A[i] @ J[i][3:] - Jr)),
            )
            err_v = max(
                err_v,
                np.max(np.abs(A[i] @ v[i][:3] - Jp @ qd)),
                np.max(np.abs(A[i] @ v[i][3:] - Jr @ qd)),
            )
        # 任务点（B12 原点）与 MuJoCo 的 jac
        bid12 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "B12")
        Jp12 = np.zeros((3, model.nv))
        Jr12 = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, Jp12, Jr12, data.xpos[bid12], bid12)
        err_Jtask = max(
            np.max(np.abs(Jtask[:3] - Jp12)),
            np.max(np.abs(Jtask[3:] - Jr12)),
        )
        print(f"  FK 位置 vs MuJoCo              : {err_r:.3e} m")
        print(f"  FK 姿态 vs MuJoCo              : {err_A:.3e}")
        print(f"  雅可比 vs MuJoCo mj_jac        : {err_J:.3e}")
        print(f"  速度 vs mj_jac @ qvel          : {err_v:.3e}")
        print(f"  任务雅可比 vs MuJoCo           : {err_Jtask:.3e}")
    else:
        print("  INFO 未安装 mujoco，跳过 MuJoCo 对照")

    # ---- 总结 ----
    tol = dict(r=1e-12, A=1e-12, J=1e-12, C=1e-12, Jc=1e-9,
               psi=1e-9, a=1e-7, cdd=1e-6)
    checks = [
        ("q=0 闭环残差 < 1e-6 m", np.max(np.linalg.norm(c0, axis=1)) < 1e-6),
        ("Jc 秩 = 8", rank_c == 8),
        ("任务雅可比秩 = 6", int(np.sum(s_t > 1e-6)) == 6),
        ("约束任务雅可比秩 = 6", int(np.sum(s_tn > 1e-6)) == 6),
        ("C 直接式 vs 连乘式", err_C_direct < tol["C"]),
        ("CΦ vs J 列", err_C_column < tol["C"]),
        ("Jc 有限差分", err_Jc < tol["Jc"]),
        ("Psi 形式", err_psi < tol["psi"]),
        ("加速度差分", err_a < tol["a"]),
        ("闭环 c_ddot 差分", err_cdd < tol["cdd"]),
    ]
    if mujoco is not None:
        checks += [
            ("FK 位置", err_r < tol["r"]),
            ("FK 姿态", err_A < tol["A"]),
            ("雅可比", err_J < tol["J"]),
            ("速度", err_v < tol["J"]),
            ("任务雅可比", err_Jtask < tol["J"]),
        ]
    print("\n[5] 结论")
    for name, ok in checks:
        print(f"  {'OK  ' if ok else 'FAIL'} {name}")
    all_ok = all(ok for _, ok in checks)
    print("-" * 78)
    print("总结论: " + ("PASS" if all_ok else "FAIL"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
