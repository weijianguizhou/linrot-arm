# 03 速度级递推与雅可比

本章推导讲义 (9.3)–(9.5) 的 6 维速度递推，给出本模型的 `\Phi_i`、`C_{i,p}`、
任意祖先的 `C_{i,k}`，以及体雅可比和 `B12` 任务雅可比。

---

## 1. 6 维速度的定义

讲义定义体 `j` 相对体 `i` 的速度为

$$
v_{i,j|j}
=
\begin{bmatrix}
A_{i,j}^T\,\dfrac{d}{dt}r_{O_iO_j|i}\\[6pt]
\omega_{i,j|j}
\end{bmatrix}
\in \mathbb{R}^6,
\tag{3.1}
$$

其中：

- 线性部分是世界系位置矢量 `r_{O_iO_j|i}` 的时间导数，再变换到 `j` 系；
- 角速度 `\omega_{i,j|j}` 是体 `j` 相对体 `i` 的角速度，投影在 `j` 系；
- 6 维顺序为 `[线; 角]`。

当 `i=0` 时，简写为

$$
v_j \equiv v_{0,j|j},
\qquad
a_j \equiv a_{0,j|j}.
\tag{3.2}
$$

注意：`v_{i,j|j}` 的线性部分不是“在 `j` 系中对时间求导”，而是“先在 `i` 系中求位置矢量的时间导数，
再变换到 `j` 系”。这个区别在加速度级会变得重要（第 4 章）。

---

## 2. 相对速度与 `\Phi_i`

相邻体 `i` 相对父体 `p=p(i)` 的速度为

$$
v_{p,i|i}
=
\begin{bmatrix}
A_{p,i}^T\,\dfrac{d}{dt}r_{O_pO_i|p}\\[6pt]
\omega_{p,i|i}
\end{bmatrix}.
\tag{3.3}
$$

### 2.1 转动铰

由 (2.2)，`r_{O_pO_i|p}=o_i` 为常矢量，所以

$$
\frac{d}{dt}r_{O_pO_i|p}=0,
\qquad
\omega_{p,i|i}=n_i\dot q_i.
$$

因此

$$
v_{p,i|i}
=
\begin{bmatrix}0_{3\times1}\\ n_i\end{bmatrix}\dot q_i
=
\Phi_i\dot q_i,
\qquad
\Phi_i=\begin{bmatrix}0\\ n_i\end{bmatrix}.
\tag{3.4}
$$

### 2.2 棱柱铰

由 (2.3)，

$$
\frac{d}{dt}r_{O_pO_i|p}=n_i\dot q_i,
\qquad
\omega_{p,i|i}=0.
$$

因此

$$
v_{p,i|i}
=
\begin{bmatrix}n_i\\0_{3\times1}\end{bmatrix}\dot q_i
=
\Phi_i\dot q_i,
\qquad
\Phi_i=\begin{bmatrix}n_i\\0\end{bmatrix}.
\tag{3.5}
$$

### 2.3 汇总

| 关节类型 | `\Phi_i` | `\xi_i` |
| :--- | :--- | :--- |
| 转动铰 | `[0; n_i]` | `0` |
| 棱柱铰 | `[n_i; 0]` | `0` |
| 固结铰 | `0` | `0` |
| 虚拟 6DOF 铰 | `diag(A_{p,i}^T,H_{B321})` | `[0; \dot H_{B321}\dot\theta]` |
| 球铰 | `[0; H_{B321}]` | `[0; \dot H_{B321}\dot\theta]` |

本模型的 18 个树关节只用到前两行；虚拟铰、球铰和固结铰的公式见附录 A。

---

## 3. 相邻体速度递推与 `C_{i,p}`

### 3.1 位置关系

由 (2.7)，

$$
r_i = r_p + A_{0,p}\rho_i,
\qquad
\rho_i = o_i+s_i,
\tag{3.6}
$$

其中 `\rho_i` 是 `O_p → O_i` 在父体系中的矢量。

### 3.2 对时间求导

对 (3.6) 求导：

$$
\dot r_i
=
\dot r_p
+
\dot A_{0,p}\rho_i
+
A_{0,p}\dot\rho_i.
\tag{3.7}
$$

利用

$$
\dot A_{0,p}=A_{0,p}[\omega_{0,p|p}]_\times,
\tag{3.8}
$$

得到

$$
\dot r_i
=
\dot r_p
+
A_{0,p}\bigl(\omega_p\times\rho_i\bigr)
+
A_{0,p}\dot\rho_i,
\tag{3.9}
$$

其中 `\omega_p=\omega_{0,p|p}`。

### 3.3 变到 `i` 系

左乘 `A_{0,i}^T=A_{p,i}^TA_{0,p}^T`：

$$
\begin{aligned}
v_i^{\text{线}}
&=
A_{p,i}^Tv_p^{\text{线}}
+
A_{p,i}^T\bigl(\omega_p\times\rho_i\bigr)
+
A_{p,i}^T\dot\rho_i\\
&=
A_{p,i}^Tv_p^{\text{线}}
+
A_{p,i}^T[\rho_i]_\times^T\omega_p
+
A_{p,i}^T\dot\rho_i.
\end{aligned}
\tag{3.10}
$$

这里用了

$$
\omega\times\rho = -[\rho]_\times\omega = [\rho]_\times^T\omega,
\qquad
[\rho]_\times^T=-[\rho]_\times.
$$

### 3.4 角速度

体 `i` 的绝对角速度在 `i` 系中为

$$
\omega_i
=
A_{p,i}^T\omega_p+\omega_{p,i|i}.
\tag{3.11}
$$

对转动铰，`\omega_{p,i|i}=n_i\dot q_i`；对棱柱铰，`\omega_{p,i|i}=0`。

### 3.5 合并成 6×6 形式

把 (3.10)、(3.11) 写成矩阵：

$$
\boxed{
C_{i,p}
=
\begin{bmatrix}
A_{p,i}^T & A_{p,i}^T[\rho_i]_\times^T\\[4pt]
0 & A_{p,i}^T
\end{bmatrix},
\qquad
v_i=C_{i,p}v_p+\Phi_i\dot q_i.
}
\tag{3.12}
$$

这就是讲义 (9.4)–(9.5) 在本模型中的具体形式。

几点说明：

1. `C_{i,p}` 是块上三角矩阵，左上、右上、右下都是 `A_{p,i}^T` 的变体，左下为零。
2. `C_{i,p}` 把父体 `p` 系的 6 维速度变换到子体 `i` 系；它不是简单的旋转矩阵，
   还包含 `O_p → O_i` 的力臂项。
3. 对转动铰，`\dot\rho_i=0`，所以 `\Phi_i\dot q_i` 只含角速度；对棱柱铰，
   `\dot\rho_i=n_i\dot q_i`，所以 `\Phi_i\dot q_i` 只含线速度。
4. `C_{i,i}=I_6`。

### 3.6 与讲义符号的对应

讲义的 `\tilde r_{O_pO_i|p}` 就是本文的 `[\rho_i]_\times`。因此

$$
C_{i,p}
=
\begin{bmatrix}
A_{p,i}^T & A_{p,i}^T\tilde r_{O_pO_i|p}^T\\
0 & A_{p,i}^T
\end{bmatrix}.
\tag{3.13}
$$

---

## 4. 任意祖先的 `C_{i,k}`

### 4.1 直接形式

对任意 `k\in\mathrm{pvec}[i]`，令

$$
r_{O_kO_i|k}=A_{0,k}^T(r_i-r_k).
\tag{3.14}
$$

则把 `k` 系速度变换到 `i` 系：

$$
\boxed{
C_{i,k}
=
\begin{bmatrix}
A_{k,i}^T & A_{k,i}^T[r_{O_kO_i|k}]_\times^T\\[4pt]
0 & A_{k,i}^T
\end{bmatrix}.
}
\tag{3.15}
$$

这是讲义 (9.51)。

### 4.2 连乘形式

若 `k\in\mathrm{pvec}[p(i)]`，则

$$
C_{i,k}=C_{i,p(i)}\,C_{p(i),k}.
\tag{3.16}
$$

从 `i` 沿父链走到 `k`，把沿途的相邻 `C` 依次相乘：

$$
C_{i,k}
=
C_{i,p(i)}C_{p(i),p(p(i))}\cdots C_{k',k},
\tag{3.17}
$$

其中 `k'` 是 `k` 在路径上的子体。这就是讲义 (9.54) 的连乘形式。

### 4.3 任意体速度

把 (3.12) 沿父链展开：

$$
\boxed{
v_i
=
\sum_{k\in\mathrm{pvec}[i]}C_{i,k}\Phi_k\dot q_k.
}
\tag{3.18}
$$

这就是讲义 (9.50) 的形式。它说明体 `i` 的速度只由它的前序关节决定；
非祖先关节的列恒为零。

---

## 5. 体雅可比 `J_i`

定义 `J_i` 为

$$
v_i=J_i(q)\dot q,
\qquad
J_i\in\mathbb{R}^{6\times18}.
\tag{3.19}
$$

由 (3.18)：

$$
\boxed{
J_i[:,k-1]=
\begin{cases}
C_{i,k}\Phi_k, & k\in\mathrm{pvec}[i],\\
0, & k\notin\mathrm{pvec}[i],
\end{cases}
}
\tag{3.20}
$$

特别地，`J_i[:,i-1]=\Phi_i`，因为 `C_{i,i}=I`。

### 5.1 递推实现

实际计算时不需要对每个 `(i,k)` 都算直接式；用

$$
J_i=C_{i,p(i)}J_{p(i)},
\qquad
J_i[:,i-1]=\Phi_i
\tag{3.21}
$$

从根往下递推即可。`J_0=0`。

### 5.2 本模型的关键链

- `J_6` 的非零列：`1..6`；
- `J_7` 的非零列：`1..7`；
- `J_13`（`B12`）的非零列：`1..6, 8..13`；
- `J_18`（`B11`）的非零列：`1..6, 14..18`。

这与第 1 章的 `pvec` 完全一致。`B12` 不受右腕链 `14..18` 的直接影响；
`B18` 不受左腕链 `8..13` 和 `B7` 的直接影响。

---

## 6. 任务雅可比

### 6.1 任务点

设任务点在体 `i` 的局部坐标为 `p`，则

$$
x=r_i+A_{0,i}p.
\tag{3.22}
$$

对时间求导：

$$
\dot x
=
\dot r_i+\dot A_{0,i}p
=
A_{0,i}\bigl(v_i^{\text{线}}+\omega_i\times p\bigr).
\tag{3.23}
$$

利用 `\omega\times p=-[p]_\times\omega`：

$$
\dot x
=
A_{0,i}
\begin{bmatrix}
I & -[p]_\times\\
0 & I
\end{bmatrix}
v_i.
\tag{3.24}
$$

定义点偏移矩阵

$$
T(p)=
\begin{bmatrix}
I & -[p]_\times\\
0 & I
\end{bmatrix}.
\tag{3.25}
$$

则

$$
\boxed{
J_T
=
\begin{bmatrix}
A_{0,i} & 0\\
0 & A_{0,i}
\end{bmatrix}
T(p)\,J_i.
}
\tag{3.26}
$$

### 6.2 角速度部分

由 (3.24)，任务角速度只取 `v_i` 的角速度部分：

$$
\omega_T=A_{0,i}\,\omega_i
=
A_{0,i}\,J_i[3:,:]\,\dot q.
\tag{3.27}
$$

所以 `J_T` 的后三行就是 `A_{0,i}J_i[3:,:]`，与 `p` 无关。

### 6.3 默认任务点

本文默认取 `B12` 原点，即 `i=13`、`p=0`：

$$
J_T=
\begin{bmatrix}
A_{0,13} & 0\\
0 & A_{0,13}
\end{bmatrix}
J_{13}.
\tag{3.28}
$$

`q=0` 时 `A_{0,13}=I`，所以 `J_T=J_{13}`。若任务点有偏移 `p_t`，只需把 `T(p_t)` 插到
(3.26) 中，位置递推和体雅可比都不变。

---

## 7. 秩与零空间

### 7.1 `B12` 任务雅可比

`q=0` 时，`J_T` 是 6×18，奇异值为

$$
[2.547556,\ 1.516733,\ 1.414653,\ 1.412111,\ 0.585672,\ 0.423217],
$$

秩为 **6**。这说明在 `q=0` 处末端有完整的 6 维运动能力。

### 7.2 闭环雅可比

`J_c` 是 9×18，`q=0` 时奇异值为

$$
[1.620434,\ 1.000403,\ 0.621119,\ 0.287159,\ 0.248281,\ 0.104507,\ 0.075249,\ 0.041458,\ 1.414214\times10^{-7}],
$$

秩为 **8**，零空间维数为 `18-8=10`。`J_c` 的构造和秩的物理解释见第 5 章。

### 7.3 约束零空间上的任务雅可比

取 `J_c` 的零空间基 `N_c\in\mathbb{R}^{18\times10}`，计算

$$
J_T N_c\in\mathbb{R}^{6\times10}.
$$

其奇异值为

$$
[2.01949,\ 1.359284,\ 1.109934,\ 0.517564,\ 0.212974,\ 0.01188],
$$

秩为 **6**，所以内部自由度数为

$$
10-6=4.
$$

最小奇异值 `0.01188` 说明 `q=0` 附近已经接近任务奇异；虽然秩仍是 6，
但某些任务方向的运动能力很弱。本文只报告这一点，不做奇异回避。

---

## 8. 数值校核

`verify_kinematics.py` 在随机位形（`seed=20261122`）下做了以下对照：

| 检查项 | 最大误差 |
| :--- | ---: |
| `C_{i,k}` 直接式 vs 连乘式 | `2.665e-15` |
| `C_{i,k}\Phi_k` vs `J_i` 第 `k` 列 | `2.554e-15` |
| `J_c` 解析 vs `c(q)` 中心差分 | `1.469e-10` |
| 体雅可比 vs MuJoCo `mj_jac` | `3.220e-15` |
| 体速度 vs `mj_jac @ qvel` | `3.331e-16` |
| 任务雅可比 vs MuJoCo | `3.220e-15` |

这些误差都在双精度舍入或有限差分截断的量级，说明：

- 相邻 `C` 的块结构正确；
- 任意祖先的连乘关系正确；
- `\Phi_i` 的转动/棱柱特化正确；
- 任务雅可比的点偏移和世界系变换正确。

---

## 9. 本章小结

- 6 维速度定义为 `[A^T\dot r;\omega]`，顺序为 `[线;角]`。
- 转动铰 `\Phi=[0;n]`，棱柱铰 `\Phi=[n;0]`。
- 相邻体递推 `v_i=C_{i,p}v_p+\Phi_i\dot q_i`，
  `C_{i,p}` 含旋转和力臂项。
- 任意祖先 `C_{i,k}` 有直接式和连乘式；体雅可比只在 `pvec[i]` 的列上非零。
- 任务雅可比 `J_T=blockdiag(A,A)T(p)J_i`；默认 `p=0` 时退化为 `blockdiag(A,A)J_i`。
- `q=0` 时 `J_T` 秩 6，`J_c` 秩 8，约束零空间 10 维，内部自由度 4。
