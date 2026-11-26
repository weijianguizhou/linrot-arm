# 附录 A 一般关节的 `\Phi` 与 `\xi`

正文的 18 个树关节只有转动铰 `R` 和棱柱铰 `P`。讲义还给出了虚拟 6 自由度铰、
球铰和固结铰的一般形式；本附录把这些形式集中列出，并说明它们与本机械臂
鱼眼链的关系。

> 本附录只补全运动学公式，不涉及控制、逆解或电机驱动。

---

## A.1 一般形式回顾

对相邻体 `p→i`，相对速度与相对加速度写成

$$
v_{p,i|i}=\Phi_i\dot q_{J_i},
\qquad
a_{p,i|i}=\Phi_i\ddot q_{J_i}+\xi_i,
\tag{A.1}
$$

其中 `q_{J_i}` 是关节 `i` 的广义坐标（可能是 1 维、3 维或 6 维），
`\Phi_i` 是相对速度基，`\xi_i` 是相对加速度偏置。

把 (A.1) 代入相邻体递推，就得到正文的

$$
v_i=C_{i,p}v_p+\Phi_i\dot q_{J_i},
\qquad
a_i=C_{i,p}a_p+\Phi_i\ddot q_{J_i}+\eta_i.
\tag{A.2}
$$

不同关节的差别全部体现在 `\Phi_i`、`\xi_i` 和 `A_{p,i}` 上。

---

## A.2 虚拟 6 自由度铰

### A.2.1 坐标与变换

虚拟 6 自由度铰用于把“浮动基”接到地面，或用 6 个参数描述一个自由刚体。
取广义坐标

$$
q_{J_i}=
\begin{bmatrix}
l\\ \theta
\end{bmatrix}
\in\mathbb{R}^6,
\qquad
l\in\mathbb{R}^3,
\qquad
\theta=
\begin{bmatrix}\theta_1\\ \theta_2\\ \theta_3\end{bmatrix}.
\tag{A.3}
$$

`l` 是子体原点相对铰点的平移，在父体系中表示；`\theta` 是 B321 欧拉角
（先绕 `z` 转 `θ1`，再绕 `y` 转 `θ2`，最后绕 `x` 转 `θ3`）。姿态变换取

$$
A_{p,i}=A_{B321}(\theta)
=R_z(\theta_1)R_y(\theta_2)R_x(\theta_3).
\tag{A.4}
$$

显式矩阵为

$$
A_{B321}=
\begin{bmatrix}
c_1c_2 & c_1s_2s_3-s_1c_3 & c_1s_2c_3+s_1s_3\\
s_1c_2 & s_1s_2s_3+c_1c_3 & s_1s_2c_3-c_1s_3\\
-s_2 & c_2s_3 & c_2c_3
\end{bmatrix},
\tag{A.5}
$$

其中 `s_k=\sin\theta_k`，`c_k=\cos\theta_k`。相对位置为

$$
r_{R_iO_i|p}=l.
\tag{A.6}
$$

### A.2.2 相对速度

对 (A.6) 求导，再变到子体系：

$$
v_{p,i|i}
=
\begin{bmatrix}
A_{p,i}^T\dot l\\
\omega_{p,i|i}
\end{bmatrix}.
\tag{A.7}
$$

B321 欧拉角速度与子体角速度的关系为

$$
\omega_{p,i|i}=H_{B321}(\theta)\dot\theta,
\tag{A.8}
$$

其中

$$
H_{B321}(\theta)=
\begin{bmatrix}
-s_2 & 0 & 1\\
c_2s_3 & c_3 & 0\\
c_2c_3 & -s_3 & 0
\end{bmatrix}.
\tag{A.9}
$$

因此

$$
\boxed{
\Phi_i=
\begin{bmatrix}
A_{p,i}^T & 0\\
0 & H_{B321}(\theta)
\end{bmatrix}
=
\operatorname{diag}\bigl(A_{p,i}^T,H_{B321}\bigr).
}
\tag{A.10}
$$

### A.2.3 `H_{B321}` 的推导

由 `[\omega]_\times=A^T\dot A`，对 (A.4) 逐项求导：

$$
\dot A=
\dot R_zR_yR_x
+R_z\dot R_yR_x
+R_zR_y\dot R_x.
\tag{A.11}
$$

利用 `R_z^T\dot R_z=[e_z]_\times\dot\theta_1`、
`R_y^T\dot R_y=[e_y]_\times\dot\theta_2`、
`R_x^T\dot R_x=[e_x]_\times\dot\theta_3`，可得

$$
A^T\dot A
=
\dot\theta_1\,A_{x}^T[e_z]_\times A_x
+\dot\theta_2\,A_{xy}^T[e_y]_\times A_{xy}
+\dot\theta_3\,[e_x]_\times,
\tag{A.12}
$$

其中 `A_x=R_x(\theta_3)`，`A_{xy}=R_y(\theta_2)R_x(\theta_3)`。
把三个叉乘矩阵展开并合并，得到

$$
[\omega_{p,i|i}]_\times
=
[H_{B321}\dot\theta]_\times,
\tag{A.13}
$$

即 (A.8)–(A.9)。这一步说明 `H_{B321}` 的列就是三个欧拉角速度对子体角速度的
瞬时贡献，且都在子体系中表示。

### A.2.4 相对加速度

对 (A.8) 求导：

$$
\dot\omega_{p,i|i}
=H_{B321}\ddot\theta+\dot H_{B321}\dot\theta.
\tag{A.14}
$$

其中

$$
\dot H_{B321}=
\begin{bmatrix}
-c_2\dot\theta_2 & 0 & 0\\
-s_2s_3\dot\theta_2+c_2c_3\dot\theta_3 & -s_3\dot\theta_3 & 0\\
-s_2c_3\dot\theta_2-c_2s_3\dot\theta_3 & -c_3\dot\theta_3 & 0
\end{bmatrix}.
\tag{A.15}
$$

注意正文的加速度定义是

$$
a=\begin{bmatrix}A^T\ddot r\\ \dot\omega\end{bmatrix},
\tag{A.16}
$$

不是 `d/dt v`。对平移部分，`r_{R_iO_i|p}=l`，所以
`A^T\ddot r_{R_iO_i}=A^T\ddot l`，没有 `\dot A^T\dot l` 项。
因此

$$
\boxed{
\xi_i=
\begin{bmatrix}
0\\
\dot H_{B321}\dot\theta
\end{bmatrix}.
}
\tag{A.17}
$$

相对加速度为

$$
a_{p,i|i}
=
\begin{bmatrix}
A_{p,i}^T\ddot l\\
H_{B321}\ddot\theta+\dot H_{B321}\dot\theta
\end{bmatrix}.
\tag{A.18}
$$

### A.2.5 奇异

`H_{B321}` 在 `\theta_2=\pm\pi/2` 时降秩（万向锁）。这是 B321 参数化的奇异，
不是机构奇异。本机械臂没有使用单个 B321 虚拟铰，而是用三个单轴转动铰串联，
因此不需要在推导中处理这个奇异。

---

## A.3 球铰

球铰只有 3 个转动自由度，没有相对平移：

$$
q_{J_i}=\theta\in\mathbb{R}^3,
\qquad
r_{R_iO_i|p}=0,
\qquad
A_{p,i}=A_{B321}(\theta).
\tag{A.19}
$$

因此

$$
\boxed{
\Phi_i=
\begin{bmatrix}
0\\
H_{B321}(\theta)
\end{bmatrix},
\qquad
\xi_i=
\begin{bmatrix}
0\\
\dot H_{B321}\dot\theta
\end{bmatrix}.
}
\tag{A.20}
$$

球铰的 `\Phi_i` 是 `6×3`，`\xi_i` 是 `6×1`。它与虚拟铰的区别只是没有
`A^T\dot l` 的平移列。

若球铰用四元数或其他 3 参数姿态表示，`H` 的形式会变，但
`\Phi_i=[0;H]`、`\xi_i=[0;\dot H\dot\theta]` 的结构不变。

---

## A.4 固结铰

固结铰没有相对自由度：

$$
\Phi_i=0_{6\times1},
\qquad
\xi_i=0_6.
\tag{A.21}
$$

若固结变换不是恒等，取固定的 `A_{p,i}` 和 `r_{R_iO_i|p}` 即可；
它们不随 `q` 变化，递推时作为常数进入 `C_{i,p}`。

本机械臂中 `B10` 与体 12 固结、`B11` 与体 18 固结、`B0` 与地面固结，
运动学上都按 (A.21) 处理，直接把固结体并入父体。

---

## A.5 转动铰与棱柱铰（正文用到的两种）

为完整起见，把正文的两种关节也列在这里。

### A.5.1 转动铰

$$
q_{J_i}=\theta,
\qquad
A_{p,i}=\operatorname{Rot}(n_i,\theta),
\qquad
r_{R_iO_i|p}=0,
\tag{A.22}
$$

$$
\boxed{
\Phi_i=
\begin{bmatrix}0\\ n_i\end{bmatrix},
\qquad
\xi_i=0.
}
\tag{A.23}
$$

### A.5.2 棱柱铰

$$
q_{J_i}=l,
\qquad
A_{p,i}=I,
\qquad
r_{R_iO_i|p}=n_i l,
\tag{A.24}
$$

$$
\boxed{
\Phi_i=
\begin{bmatrix}n_i\\ 0\end{bmatrix},
\qquad
\xi_i=0.
}
\tag{A.25}
$$

### A.5.3 汇总

| 关节 | `q` 维数 | `A_{p,i}` | `r_{R_iO_i|p}` | `\Phi_i` | `\xi_i` |
| :--- | ---: | :--- | :--- | :--- | :--- |
| 转动 `R` | 1 | `Rot(n,\theta)` | `0` | `[0;n]` | `0` |
| 棱柱 `P` | 1 | `I` | `n l` | `[n;0]` | `0` |
| 固结 | 0 | 固定 | 固定 | `0` | `0` |
| 球铰 | 3 | `A_{B321}(\theta)` | `0` | `[0;H]` | `[0;\dot H\dot\theta]` |
| 虚拟 6DOF | 6 | `A_{B321}(\theta)` | `l` | `diag(A^T,H)` | `[0;\dot H\dot\theta]` |

---

## A.6 本机械臂的鱼眼链

### A.6.1 为什么不用单个球铰

本机械臂的左右腕部各有一个“鱼眼”式三轴关节，但模型把它展开成 3 个串联的
单轴转动铰：

- 左腕：`L2g_gy`（`Y`）→ `L2g_gx`（`X`）→ `L2g_gz`（`Z`），体 10→11→12；
- 右腕：`L3g_gy`（`Y`）→ `L3g_gx`（`X`）→ `L3g_gz`（`Z`），体 16→17→18。

每个铰都有自己的 `q`、`\Phi=[0;n]`、`\xi=0`，按正文的递推逐铰计算。
这样做的好处是：

1. 每个关节的限位、被动/主动属性和雅可比列都单独可见；
2. 不需要为 Y-X-Z 序列重新推导一个闭合的 `H`；
3. 不会引入 B321 的万向锁奇异。

### A.6.2 复合 `\Phi`

虽然实现时逐铰递推，仍可以把三个转动铰的瞬时贡献合成为一个 `6×3` 的
复合 `\Phi`。对左腕，写

$$
\Phi_{L2g}
=
\begin{bmatrix}
C_{12,10}\Phi_{10} &
C_{12,11}\Phi_{11} &
\Phi_{12}
\end{bmatrix}
\in\mathbb{R}^{6\times3}.
\tag{A.26}
$$

对右腕：

$$
\Phi_{L3g}
=
\begin{bmatrix}
C_{18,16}\Phi_{16} &
C_{18,17}\Phi_{17} &
\Phi_{18}
\end{bmatrix}
\in\mathbb{R}^{6\times3}.
\tag{A.27}
$$

其中 `C_{i,k}` 是正文 (3.15)–(3.17) 的任意祖先变换。由于 `\Phi_k=[0;n_k]`，
复合 `\Phi` 的后三行就是三个轴在末端体系中的表示：

$$
H_{L2g}
=
\begin{bmatrix}
A_{10,12}^Tn_{10} &
A_{11,12}^Tn_{11} &
n_{12}
\end{bmatrix},
\qquad
H_{L3g}
=
\begin{bmatrix}
A_{16,18}^Tn_{16} &
A_{17,18}^Tn_{17} &
n_{18}
\end{bmatrix}.
\tag{A.28}
$$

这里 `A_{k,i}^Tn_k` 把轴 `n_k` 从体 `k` 变到体 `i`。这个 `H` 不是 B321 的
`H_{B321}`，因为转轴顺序是 `Y-X-Z` 而不是 `Z-Y-X`；它由三个单轴旋转复合而成，
一般没有简单的三行闭式，但递推 (A.26)–(A.28) 已经足够。

### A.6.3 与球铰的差别

如果把这个鱼眼链近似成一个球铰，可以取 `\Phi≈[0;H_{L2g}]`、
`\xi≈[0;\dot H_{L2g}\dot\theta]`。但本文不这样做：

- 3 个铰是真实的树关节，各自有独立的 `q` 和限位；
- 闭环约束的雅可比需要每个铰的单独列；
- 近似成球铰会丢掉三个铰的限位和被动属性。

因此正文和脚本都按 3 个单轴转动铰逐铰递推，附录只给出复合形式作为对照。

---

## A.7 与讲义公式的对应

| 讲义 | 内容 | 本文位置 |
| :--- | :--- | :--- |
| 虚拟铰 | `Φ=diag(A^T,H_{B321})`，`ξ=[0;\dot H\dot\theta]` | (A.10)、(A.17) |
| 球铰 | `Φ=[0;H_{B321}]`，`ξ=[0;\dot H\dot\theta]` | (A.20) |
| 固结铰 | `Φ=0`，`ξ=0` | (A.21) |
| 转动铰 | `Φ=[0;n]`，`ξ=0` | (A.23) |
| 棱柱铰 | `Φ=[n;0]`，`ξ=0` | (A.25) |
| 鱼眼链 | 三个单轴转动铰串联 | (A.26)–(A.28) |

`H_{B321}` 和 `\dot H_{B321}` 的显式形式见 (A.9)、(A.15)。
正文的 18 个树关节只用到 (A.23)、(A.25) 和固结铰的合并规则；
虚拟铰、球铰的公式用于说明讲义的一般性和鱼眼链的对照。
