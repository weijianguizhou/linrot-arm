# 06 系统运动学方程汇总

前四章分别给出了位置、速度、加速度和闭环约束的递推式。本章把它们组装成
一套完整的系统方程，并给出维数、自由度分解和适用范围。

> 本章只做运动学关系的整理，不包含控制算法、逆解迭代、轨迹生成或电机驱动。
> 所有“解集”“零空间”的说法都是线性代数的描述，不表示本文给出了求解算法。

---

## 1. 系统描述与广义坐标

### 1.1 派生树

机械臂在运动学上化为 **18 个运动体 + 18 个树关节 + 3 个闭环切断铰**：

- 体编号 `1..18`，体 `i` 由关节 `i` 驱动；
- 父体编号满足 `p(i) < i`，`p(1)=0`，体 `0` 为地面；
- 每个树关节只有一个广义坐标 `q_i`，因此

$$
q=\begin{bmatrix}q_1&\cdots&q_{18}\end{bmatrix}^T\in\mathbb{R}^{18}.
\tag{6.1}
$$

父数组为

$$
p=\begin{bmatrix}
-1,&0,&1,&2,&3,&4,&5,&6,&6,&8,&9,&10,&11,&12,&6,&14,&15,&16,&17
\end{bmatrix}.
\tag{6.2}
$$

前序集合 `pvec[i]` 由 `p` 唯一确定，体 `i` 的位置、速度和加速度只依赖
`pvec[i]` 上的坐标。拓扑细节见 [01-topology.md](01-topology.md)。

### 1.2 执行器坐标与被动坐标

6 个执行器关节为

$$
\mathcal{A}=\{R1,R2,R3,L1,L2,L3\}
=\{1,2,3,5,9,15\},
\tag{6.3}
$$

其余 12 个关节为被动关节。定义选择矩阵

$$
S_a\in\mathbb{R}^{6\times18},\qquad
S_p\in\mathbb{R}^{12\times18},
\tag{6.4}
$$

分别为单位阵中对应行组成的矩阵，则

$$
q_a=S_aq\in\mathbb{R}^{6},\qquad
q_p=S_pq\in\mathbb{R}^{12}.
\tag{6.5}
$$

需要强调：`q_a` 是 6 个执行器坐标，不等于 6 个独立自由度。
系统的独立坐标数是 10，见第 6、7 节。

### 1.3 闭环

3 个闭环在派生树中被切断，作为切断铰处理：

| 闭环 | 父体 `b` | 子体 `m` | 参考体 `f` |
| :--- | ---: | ---: | ---: |
| `R6` | 3 (`B3`) | 6 (`B6`) | 3 |
| `R10` | 7 (`B7`) | 13 (`B12`) | 6 |
| `R12` | 18 (`B11`) | 13 (`B12`) | 6 |

每个闭环贡献 3 行位置约束，共 9 行。闭环几何与约束推导见
[05-closed-loop.md](05-closed-loop.md)。

---

## 2. 位置级方程

### 2.1 单铰变换

对转动铰 `R`：

$$
A_{p,i}=\operatorname{Rot}(n_i,q_i)
=I\cos q_i+[n_i]_\times\sin q_i
+n_in_i^T(1-\cos q_i),
\qquad
s_i=0.
\tag{6.6}
$$

对棱柱铰 `P`：

$$
A_{p,i}=I,\qquad s_i=n_iq_i.
\tag{6.7}
$$

其中 `o_i` 是关节轴点在父体系中的位置，`n_i` 是父体系中的单位轴。
`q=0` 时所有体坐标系与世界系同向。

### 2.2 位置递推

从 `r_0=0`、`A_{0,0}=I` 出发，按 `i=1..18` 递推：

$$
\boxed{
r_i=r_{p(i)}+A_{0,p(i)}\bigl(o_i+s_i\bigr),
\qquad
A_{0,i}=A_{0,p(i)}A_{p(i),i}.
}
\tag{6.8}
$$

其中 `r_i` 是体 `i` 原点在世界系中的位置，`A_{0,i}` 是体 `i` 到世界系的旋转矩阵。
把 18 个体的位置堆叠起来，记为

$$
r(q)=\begin{bmatrix}r_1\\ \vdots\\ r_{18}\end{bmatrix}\in\mathbb{R}^{54}.
\tag{6.9}
$$

### 2.3 任务位姿

取 `B12`（体 13）为任务体，任务点在 `B12` 局部坐标为 `p_t`（默认 `p_t=0`）。
任务点位置与任务姿态为

$$
r_T=r_{13}+A_{0,13}p_t,
\qquad
R_T=A_{0,13}R_t,
\tag{6.10}
$$

其中 `R_t` 是工具系相对 `B12` 的固定姿态偏移；默认工具系与 `B12` 同向，`R_t=I`。
任务位姿记为

$$
X_T=(r_T,R_T)\in\mathbb{R}^3\times SO(3).
\tag{6.11}
$$

`X_T` 是 6 维位姿，但它不是 `R^6` 中的向量；速度级用 6 维速度
`[\dot r_T;\omega_T]` 表示，避免姿态参数化的奇异。

### 2.4 闭环位置约束

对闭环 `j`，记父体 `b_j` 上的锚点为 `p_j^B`，子体 `m_j` 上的锚点为 `p_j^M`
（均为各自的局部坐标）。采用讲义定义

$$
c_j(q)
=r_{m_j}+A_{0,m_j}p_j^M
-\Bigl(r_{b_j}+A_{0,b_j}p_j^B\Bigr)
\in\mathbb{R}^3.
\tag{6.12}
$$

把 3 个闭环堆叠：

$$
c(q)=\begin{bmatrix}c_{R6}\\ c_{R10}\\ c_{R12}\end{bmatrix}
\in\mathbb{R}^{9},
\qquad
\boxed{c(q)=0.}
\tag{6.13}
$$

`c=0` 表示每对锚点在世界系中重合。由于模型数值的舍入，`q=0` 时
`c` 的残差约为 `4.38e-7 m`，不是严格零；理论方程按名义几何取 `c=0`。
数值见 [07-numerics.md](07-numerics.md)。

---

## 3. 速度级方程

### 3.1 体速度

体 `i` 的绝对 6 维速度定义为

$$
v_i=v_{0,i|i}
=\begin{bmatrix}
A_{0,i}^T\dot r_i\\
\omega_{0,i|i}
\end{bmatrix}
\in\mathbb{R}^{6},
\tag{6.14}
$$

顺序为 `[线速度; 角速度]`，且线速度投影在体 `i` 系。

### 3.2 相邻体递推

$$
\boxed{
v_i=C_{i,p(i)}v_{p(i)}+\Phi_i\dot q_i,
}
\tag{6.15}
$$

其中

$$
C_{i,p}=
\begin{bmatrix}
A_{p,i}^T & A_{p,i}^T[\rho_i]_\times^T\\
0 & A_{p,i}^T
\end{bmatrix},
\qquad
\rho_i=o_i+s_i,
\tag{6.16}
$$

$$
\Phi_i=
\begin{cases}
\begin{bmatrix}0\\ n_i\end{bmatrix}, & \text{转动铰},\\[6pt]
\begin{bmatrix}n_i\\ 0\end{bmatrix}, & \text{棱柱铰}.
\end{cases}
\tag{6.17}
$$

`C_{i,p}` 的推导见 [03-velocity.md](03-velocity.md) 第 3 节。

### 3.3 体雅可比

把 `v_i` 对 `\dot q` 线性化：

$$
v_i=J_i(q)\dot q,
\qquad
J_i=\begin{bmatrix}J_{i,1}&\cdots&J_{i,18}\end{bmatrix}\in\mathbb{R}^{6\times18}.
\tag{6.18}
$$

递推为

$$
J_i=C_{i,p(i)}J_{p(i)},
\qquad
J_{i,i}=\Phi_i,
\qquad
J_{i,k}=0\quad(k\notin\operatorname{pvec}[i]).
\tag{6.19}
$$

等价地，对任意祖先 `k∈pvec[i]`：

$$
J_{i,k}=C_{i,k}\Phi_k,
\qquad
C_{i,k}=C_{i,p(i)}C_{p(i),k},
\qquad
C_{i,i}=I_6.
\tag{6.20}
$$

把 18 个体雅可比堆叠：

$$
v=J(q)\dot q,
\qquad
v=\begin{bmatrix}v_1\\ \vdots\\ v_{18}\end{bmatrix}\in\mathbb{R}^{108},
\qquad
J=\begin{bmatrix}J_1\\ \vdots\\ J_{18}\end{bmatrix}\in\mathbb{R}^{108\times18}.
\tag{6.21}
$$

### 3.4 任务速度

任务点的 6 维速度为

$$
\dot x_T=\begin{bmatrix}\dot r_T\\ \omega_T\end{bmatrix}
=J_T(q)\dot q,
\tag{6.22}
$$

$$
\boxed{
J_T=
\begin{bmatrix}A_{0,13}&0\\0&A_{0,13}\end{bmatrix}
T(p_t)J_{13},
\qquad
T(p)=\begin{bmatrix}I&-[p]_\times\\0&I\end{bmatrix}.
}
\tag{6.23}
$$

默认 `p_t=0` 时 `T(p_t)=I`，`J_T=\operatorname{diag}(A_{0,13},A_{0,13})J_{13}`。
若只关心位置、不关心姿态，取 `J_T` 的前 3 行即可。

### 3.5 闭环速度约束

对 (6.13) 求导：

$$
\dot c=J_c(q)\dot q,
\tag{6.24}
$$

其中第 `j` 块为

$$
J_{c,j}
=A_{0,m_j}\begin{bmatrix}I&-[p_j^M]_\times\end{bmatrix}J_{m_j}[:3]
-A_{0,b_j}\begin{bmatrix}I&-[p_j^B]_\times\end{bmatrix}J_{b_j}[:3].
\tag{6.25}
$$

`J_c∈R^{9×18}`。闭环速度为

$$
\boxed{
J_c(q)\dot q=0.
}
\tag{6.26}
$$

因此 容许 速度属于 `ker J_c(q)`：

$$
\dot q\in\ker J_c(q),
\qquad
\dim\ker J_c(q)=18-\operatorname{rank}J_c(q).
\tag{6.27}
$$

`q=0` 时 `rank J_c=8`，所以速度自由度是 10。

---

## 4. 加速度级方程

### 4.1 体加速度

体 `i` 的绝对 6 维加速度定义为

$$
a_i=a_{0,i|i}
=\begin{bmatrix}
A_{0,i}^T\ddot r_i\\
\dot\omega_{0,i|i}
\end{bmatrix}
\in\mathbb{R}^{6}.
\tag{6.28}
$$

注意它不是 `d v_i/dt`：`v_i` 的线速度部分带了 `A_{0,i}^T`，
求导会多出 `-\omega_i\times` 项。两者的关系见
[04-acceleration.md](04-acceleration.md) 第 1.1 节。

### 4.2 相邻体递推

$$
\boxed{
a_i=C_{i,p(i)}a_{p(i)}+\Phi_i\ddot q_i+\eta_i,
}
\tag{6.29}
$$

其中

$$
\eta_i=\xi_i+
\begin{bmatrix}
A_{p,i}^T\bigl(
[\omega_p]_\times^2\rho_i
+2[\omega_p]_\times\dot\rho_i
\bigr)\\[4pt]
A_{p,i}^T[\omega_p]_\times\omega_{p,i|p}
\end{bmatrix},
\tag{6.30}
$$

`ω_p=ω_{0,p|p}`，`ρ_i=o_i+s_i`，`\dot ρ_i` 对转动铰为零、对棱柱铰为
`n_i\dot q_i`。关节相对加速度偏置为

$$
\xi_i=
\begin{cases}
0, & \text{转动铰/棱柱铰},\\
\begin{bmatrix}0\\ \dot H_{B321}\dot\theta\end{bmatrix}, & \text{球铰/虚拟铰}.
\end{cases}
\tag{6.31}
$$

本手臂的树关节只有转动铰和棱柱铰，`ξ_i=0`；鱼眼由 3 个单轴转动铰串联，
逐铰递推，不需要 B321 复合公式。一般铰的 `Φ,ξ` 见
[appendix-a-general-joints.md](appendix-a-general-joints.md)。

### 4.3 体加速度的雅可比形式

由 (6.29) 和 (6.19) 可把 `a_i` 写成

$$
a_i=J_i(q)\ddot q+b_i(q,\dot q),
\tag{6.32}
$$

其中 `b_i` 是 `\ddot q=0` 时的偏置加速度：

$$
b_i=C_{i,p(i)}b_{p(i)}+\eta_i,
\qquad
b_0=0.
\tag{6.33}
$$

堆叠后：

$$
\boxed{
a=J(q)\ddot q+b(q,\dot q),
}
\qquad
b=\begin{bmatrix}b_1\\ \vdots\\ b_{18}\end{bmatrix}\in\mathbb{R}^{108}.
\tag{6.34}
$$

### 4.4 任务加速度

对任务点 `p_t`：

$$
\ddot x_T=\begin{bmatrix}\ddot r_T\\ \dot\omega_T\end{bmatrix}
=J_T(q)\ddot q+b_T(q,\dot q),
\tag{6.35}
$$

$$
\boxed{
b_T=
\begin{bmatrix}A_{0,13}&0\\0&A_{0,13}\end{bmatrix}
\left(
T(p_t)b_{13}
+
\begin{bmatrix}
[\omega_{13}]_\times^2p_t\\0
\end{bmatrix}
\right).
}
\tag{6.36}
$$

默认 `p_t=0` 时

$$
b_T=\begin{bmatrix}A_{0,13}&0\\0&A_{0,13}\end{bmatrix}b_{13}.
\tag{6.37}
$$

### 4.5 闭环加速度约束

对 (6.26) 再求导：

$$
\ddot c=J_c(q)\ddot q+b_c(q,\dot q),
\tag{6.38}
$$

其中

$$
b_c=\dot J_c(q,\dot q)\dot q
=\begin{bmatrix}b_{c,R6}\\ b_{c,R10}\\ b_{c,R12}\end{bmatrix}
\in\mathbb{R}^{9}.
\tag{6.39}
$$

第 `j` 块的显式形式为

$$
b_{c,j}
=A_{0,m_j}\Bigl(
T(p_j^M)b_{m_j}
+\begin{bmatrix}[\omega_{m_j}]_\times^2p_j^M\\0\end{bmatrix}
\Bigr)[:3]
-A_{0,b_j}\Bigl(
T(p_j^B)b_{b_j}
+\begin{bmatrix}[\omega_{b_j}]_\times^2p_j^B\\0\end{bmatrix}
\Bigr)[:3].
\tag{6.40}
$$

闭环加速度约束为

$$
\boxed{
J_c(q)\ddot q+b_c(q,\dot q)=0.
}
\tag{6.41}
$$

---

## 5. 整体运动学方程

把第 2–4 节的结果放在一起，得到本机械臂的完整运动学方程组：

$$
\boxed{
\begin{aligned}
& r_i=r_{p(i)}+A_{0,p(i)}(o_i+s_i),
&& A_{0,i}=A_{0,p(i)}A_{p(i),i},
&& i=1,\ldots,18,\\[2pt]
& c(q)=0,
&& c\in\mathbb{R}^{9},\\[2pt]
& v=J(q)\dot q,
&& \dot x_T=J_T(q)\dot q,
&& J_c(q)\dot q=0,\\[2pt]
& a=J(q)\ddot q+b(q,\dot q),
&& \ddot x_T=J_T(q)\ddot q+b_T(q,\dot q),\\[2pt]
& J_c(q)\ddot q+b_c(q,\dot q)=0.
\end{aligned}
}
\tag{6.42}
$$

各式的含义：

1. **位置级**：由 `q` 递推所有体的 `r_i,A_{0,i}`，并满足 9 行闭环位置约束。
2. **速度级**：体速度、任务速度和闭环速度都是 `\dot q` 的线性函数；
   闭环速度约束把 容许 `\dot q` 限制在 10 维的 `ker J_c` 中。
3. **加速度级**：体加速度、任务加速度和闭环加速度都是 `\ddot q` 的仿射函数；
   闭环加速度约束把 `\ddot q` 限制在一个 10 维仿射空间中。

若只关心末端任务，方程 (6.42) 中与任务直接相关的部分是

$$
\dot x_T=J_T(q)\dot q,
\qquad
\ddot x_T=J_T(q)\ddot q+b_T(q,\dot q),
\tag{6.43}
$$

同时 `\dot q` 和 `\ddot q` 还要满足 (6.26)、(6.41)。

---

## 6. 位形空间、切空间与任务映射

### 6.1 位形空间

闭环位置约束定义位形流形

$$
\mathcal{M}=\{q\in\mathbb{R}^{18}\mid c(q)=0\}.
\tag{6.44}
$$

在 `q=0` 处 `rank J_c=8`，由隐函数定理，`\mathcal{M}` 局部是 10 维流形：

$$
\dim\mathcal{M}=18-8=10.
\tag{6.45}
$$

### 6.2 切空间

对 (6.44) 求导， 容许 速度属于切空间

$$
T_q\mathcal{M}=\ker J_c(q)
=\{\dot q\in\mathbb{R}^{18}\mid J_c(q)\dot q=0\}.
\tag{6.46}
$$

取 `J_c` 的零空间正交基

$$
N_c\in\mathbb{R}^{18\times10},
\qquad
J_cN_c=0,
\qquad
N_c^TN_c=I_{10}.
\tag{6.47}
$$

则任意 容许 速度可写成

$$
\dot q=N_cz,
\qquad
z\in\mathbb{R}^{10}.
\tag{6.48}
$$

这是运动学参数化，不涉及任何控制律。

### 6.3 任务速度映射

把 (6.48) 代入任务速度：

$$
\dot x_T=J_TN_cz,
\qquad
J_TN_c\in\mathbb{R}^{6\times10}.
\tag{6.49}
$$

`q=0` 时 `J_TN_c` 的奇异值为

$$
[2.01949,\ 1.359284,\ 1.109934,\ 0.517564,\ 0.212974,\ 0.01188],
\tag{6.50}
$$

秩为 6。因此任务速度映射在 `ker J_c` 上是满秩的，
内部速度子空间维数为

$$
10-6=4.
\tag{6.51}
$$

这 4 维就是机械臂的 **内部自由度**：它们改变 18 个树坐标和闭环内力/姿态，
但不改变末端 `B12` 的 6 维位姿。

### 6.4 加速度约束的解集

加速度约束 (6.41) 是 `\ddot q` 的仿射方程。`rank J_c=8` 时，
其解集是一个 10 维仿射空间：

$$
\mathcal{A}_q
=\{\ddot q\in\mathbb{R}^{18}\mid J_c\ddot q=-b_c\},
\qquad
\dim\mathcal{A}_q=10.
\tag{6.52}
$$

若同时指定任务加速度 `\ddot x_T`，则联立

$$
\begin{bmatrix}J_c\\J_T\end{bmatrix}\ddot q
=
\begin{bmatrix}-b_c\\ \ddot x_T-b_T\end{bmatrix}.
\tag{6.53}
$$

`q=0` 时堆叠矩阵是 `15×18`，秩为

$$
\operatorname{rank}\begin{bmatrix}J_c\\J_T\end{bmatrix}=8+6=14,
\tag{6.54}
$$

所以满足闭环约束和给定任务加速度的 `\ddot q` 有

$$
18-14=4
\tag{6.55}
$$

维的解集。这个 4 维解集与 (6.51) 的内部自由度一致。
本文只给出这个线性代数结论，不讨论如何选取其中某个解。

---

## 7. 自由度分解与执行器关系

### 7.1 维数总表

| 对象 | 符号 | 维数 | 说明 |
| :--- | :--- | ---: | :--- |
| 树坐标 | `q` | 18 | 18 个树关节 |
| 闭环位置约束 | `c` | 9 | 3 环 × 3 行 |
| 闭环独立约束 | — | 8 | `rank J_c=8` |
| 位形自由度 | — | 10 | `18-8` |
| 任务位姿 | `X_T` | 6 | `R^3×SO(3)` |
| 任务速度 | `\dot x_T` | 6 | `[\dot r_T;\omega_T]` |
| 内部自由度 | — | 4 | `10-6` |
| 执行器坐标 | `q_a` | 6 | `R1,R2,R3,L1,L2,L3` |
| 被动坐标 | `q_p` | 12 | 其余关节 |

### 7.2 为什么 6 个执行器不是 6 个自由度

给定 6 个执行器坐标 `q_a` 后，剩下 12 个被动坐标 `q_p` 要满足 8 个独立的
闭环位置约束。局部的解集维数是

$$
12-8=4.
\tag{6.56}
$$

也就是说，仅凭 6 个执行器坐标，纯运动学不能唯一确定 12 个被动关节；
一般会有 4 维的被动位形族。这个 4 与 (6.51) 的内部自由度一致。

反过来，给定末端位姿 `X_T`（6 维）和 8 个闭环约束，18 个树坐标的局部解集
一般也是 4 维：

$$
18-8-6=4.
\tag{6.57}
$$

这两个 4 维族是同一件事的两种说法：**6 个执行器驱动 10 个独立坐标，
末端只约束其中 6 个，剩下 4 个内部自由度由动力学、关节柔性或额外约束决定。**
本文不讨论这 4 个内部自由度如何被确定，也不给出任何控制或优化算法。

### 7.3 与“6 自由度臂”的说法

通常说这是一条“6 自由度机械臂”，指的是 **末端任务空间是 6 维**，
而不是说整条臂只有 6 个独立坐标。完整的运动学模型是 18 个树坐标、
8 个独立闭环约束、10 个位形自由度；末端任务映射在 `ker J_c` 上秩为 6。
这两个“6”含义不同，推导和数值校核中必须区分。

---

## 8. 一致性与适用范围

### 8.1 三级约束的一致性

由 (6.13)、(6.26)、(6.41)：

$$
c(q)=0
\;\Longrightarrow\;
\dot c=J_c\dot q=0
\;\Longrightarrow\;
\ddot c=J_c\ddot q+b_c=0.
\tag{6.58}
$$

因此闭环约束在位置、速度、加速度三级是同一约束的逐次求导，不是三个独立假设。
数值上，`b_c` 就是 `\dot J_c\dot q`；脚本用位置二阶差分校核 `\ddot c`，
误差 `1.641e-8`，见 [05-closed-loop.md](05-closed-loop.md) 第 8 节。

### 8.2 `\Psi` 形式与 `J_c` 形式

第 5 章还给出了在参考体 `f` 系中表示的 `\Psi` 形式：

$$
\dot c|_f=\Psi_mv_{m|f}-\Psi_bv_{b|f}.
\tag{6.59}
$$

当 `c=0` 时，`\Psi` 形式与 `J_c` 形式通过 `A_{0,f}^T` 等价：

$$
\dot c|_f=A_{0,f}^T\dot c.
\tag{6.60}
$$

当 `c≠0` 时，两者相差 `-\omega_f\times c|_f`。本推导在理论方程中取 `c=0`，
数值校核时用 `c_f(q)=A_{0,f}^Tc(q)` 的中心差分对照，避免把参考系旋转项
误当成约束误差。细节见 [05-closed-loop.md](05-closed-loop.md) 第 5.4 节。

### 8.3 假设

本推导采用以下假设：

1. 所有体为刚体，所有关节为理想铰，无间隙、无柔性、无摩擦。
2. 关节轴 `n_i` 在父体系中固定；关节运动不改变轴的方向。
3. `B0` 与地面固结，`B10`、`B11` 分别与体 12、体 18 固结。
4. 闭环锚点按 `model/loops.yaml` 的名义几何取值；`q=0` 的微小残差
   （最大 `4.38e-7 m`）来自模型数值舍入，不作为理论约束的一部分。
5. 闭环按球铰/鱼眼处理，每环 3 行位置约束；不采用刚性转动铰的 5 行模型。
6. 不包含碰撞、接触、电机电气动态、传动柔性和关节限位切换。

### 8.4 奇异与数值条件

`q=0` 时：

- `J_T` 秩 6，奇异值最小 `0.4232`；
- `J_TN_c` 秩 6，但最小奇异值 `0.01188`，已经接近任务奇异；
- `J_c` 的名义秩 8，最小非零奇异值 `0.04146`，另有一个
  `1.414e-7` 的数值零（对应 `R6` 的平面冗余行）。

因此 `q=0` 附近虽然秩仍是 6，某些任务方向的运动能力很弱；
闭环雅可比也接近降秩。本文只报告这些数值事实，不做奇异回避或控制设计。
完整奇异值表见 [07-numerics.md](07-numerics.md)。

---

## 9. 本章小结

- 系统由 18 个树坐标 `q`、9 行闭环位置约束 `c(q)=0` 描述；
  `q=0` 处 `rank J_c=8`，位形自由度 10。
- 位置级是 (6.8)；速度级是 `v=J\dot q`、`\dot x_T=J_T\dot q`、`J_c\dot q=0`；
  加速度级是 `a=J\ddot q+b`、`\ddot x_T=J_T\ddot q+b_T`、
  `J_c\ddot q+b_c=0`。
- 这四组方程构成完整的整体运动学方程 (6.42)。
- 末端任务 6 维，内部自由度 4 维；6 个执行器坐标不能唯一确定 12 个被动坐标，
  纯运动学留下 4 维被动位形族。
- 闭环约束在位置、速度、加速度三级一致；`\Psi` 形式与 `J_c` 形式在 `c=0`
  时等价，`c≠0` 时相差参考系旋转项。
- 本推导只描述运动学，不包含控制、逆解、轨迹生成或电机驱动。

---

## 附：本章公式与前面章节的对应

| 本章 | 内容 | 来源 |
| :--- | :--- | :--- |
| (6.6)–(6.8) | 单铰变换与位置递推 | [02-position.md](02-position.md) |
| (6.14)–(6.21) | 体速度、`C`、`\Phi`、体雅可比 | [03-velocity.md](03-velocity.md) |
| (6.22)–(6.23) | 任务雅可比 | [03-velocity.md](03-velocity.md) 第 6 节 |
| (6.24)–(6.27) | 闭环速度约束 | [05-closed-loop.md](05-closed-loop.md) 第 4 节 |
| (6.28)–(6.36) | 加速度递推、`b_i`、任务偏置 | [04-acceleration.md](04-acceleration.md) |
| (6.38)–(6.41) | 闭环加速度约束 | [05-closed-loop.md](05-closed-loop.md) 第 6 节 |
| (6.44)–(6.55) | 位形空间、零空间、自由度分解 | 本章整理；数值见 [07-numerics.md](07-numerics.md) |
