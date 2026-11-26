# 04 加速度级递推

本章推导讲义 (9.13)–(9.15) 的加速度递推。关键点是：
**加速度的线性部分是 `A^T\ddot r`，不是线速度的普通时间导数**。
把这个定义和速度递推结合，才能得到正确的 `\eta_i`。

---

## 1. 6 维加速度的定义

讲义定义

$$
a_{i,j|j}
=
\begin{bmatrix}
A_{i,j}^T\,\dfrac{d^2}{dt^2}r_{O_iO_j|i}\\[6pt]
\dfrac{d}{dt}\omega_{i,j|j}
\end{bmatrix}
\in \mathbb{R}^6.
\tag{4.1}
$$

当 `i=0` 时简写为 `a_j\equiv a_{0,j|j}`。

### 1.1 与 `d/dt v` 的区别

由 (3.1)，

$$
v_j^{\text{线}}=A_{0,j}^T\dot r_j.
$$

对时间求导：

$$
\frac{d}{dt}v_j^{\text{线}}
=
A_{0,j}^T\ddot r_j
+
\dot A_{0,j}^T\dot r_j.
$$

利用 `\dot A_{0,j}=A_{0,j}[\omega_j]_\times`，有

$$
\dot A_{0,j}^T=-[\omega_j]_\times A_{0,j}^T,
$$

所以

$$
\frac{d}{dt}v_j^{\text{线}}
=
a_j^{\text{线}}-\omega_j\times v_j^{\text{线}}.
\tag{4.2}
$$

等价地，

$$
a_j^{\text{线}}
=
\frac{d}{dt}v_j^{\text{线}}+\omega_j\times v_j^{\text{线}}.
\tag{4.3}
$$

**因此不能把 `a_j` 直接当成 `d/dt v_j`。** 第 7 节的数值校核用的是

$$
\frac{d}{dt}\bigl(A_{0,j}v_j\bigr)=A_{0,j}a_j,
\tag{4.4}
$$

这个恒等式避开了 (4.2) 的修正项。

### 1.2 角速度部分

角速度部分就是普通的体角速度导数：

$$
a_j^{\text{角}}=\frac{d}{dt}\omega_j.
\tag{4.5}
$$

---

## 2. 相对加速度

相邻体 `i` 相对父体 `p` 的加速度为

$$
a_{p,i|i}
=
\begin{bmatrix}
A_{p,i}^T\,\dfrac{d^2}{dt^2}r_{O_pO_i|p}\\[6pt]
\dfrac{d}{dt}\omega_{p,i|i}
\end{bmatrix}.
\tag{4.6}
$$

### 2.1 转动铰

`r_{O_pO_i|p}=o_i` 为常矢量，`\omega_{p,i|i}=n_i\dot q_i`，所以

$$
a_{p,i|i}
=
\begin{bmatrix}0\\ n_i\end{bmatrix}\ddot q_i
=
\Phi_i\ddot q_i.
\tag{4.7}
$$

### 2.2 棱柱铰

`r_{O_pO_i|p}=o_i+n_iq_i`，`\omega_{p,i|i}=0`，所以

$$
a_{p,i|i}
=
\begin{bmatrix}n_i\\0\end{bmatrix}\ddot q_i
=
\Phi_i\ddot q_i.
\tag{4.8}
$$

### 2.3 一般形式

对转动铰和棱柱铰，相对加速度都等于 `\Phi_i\ddot q_i`，没有偏置项：

$$
a_{p,i|i}=\Phi_i\ddot q_i+\xi_i,
\qquad
\xi_i=0.
\tag{4.9}
$$

虚拟 6DOF 铰、球铰的 `\xi_i` 不为零，见附录 A。

---

## 3. `\eta_i` 的推导

### 3.1 位置关系

由 (3.6)，

$$
r_i=r_p+A_{0,p}\rho_i,
\qquad
\rho_i=o_i+s_i.
\tag{4.10}
$$

### 3.2 一阶导

由 (3.9)：

$$
\dot r_i
=
\dot r_p
+
A_{0,p}\bigl(\omega_p\times\rho_i\bigr)
+
A_{0,p}\dot\rho_i,
\tag{4.11}
$$

其中 `\omega_p=\omega_{0,p|p}`。

### 3.3 二阶导

对 (4.11) 再求导。记 `\alpha_p=\dot\omega_p`（在 `p` 系中），则

$$
\frac{d}{dt}\bigl(\omega_p\times\rho_i\bigr)
=
\alpha_p\times\rho_i+\omega_p\times\dot\rho_i,
$$

并且

$$
\frac{d}{dt}\bigl(A_{0,p}u\bigr)
=
A_{0,p}\bigl(\omega_p\times u+\dot u\bigr).
$$

因此

$$
\ddot r_i
=
\ddot r_p
+
A_{0,p}\Bigl(
[\omega_p]_\times^2\rho_i
+
[\alpha_p]_\times\rho_i
+
2[\omega_p]_\times\dot\rho_i
+
\ddot\rho_i
\Bigr).
\tag{4.12}
$$

这里 `[\omega]_\times^2\rho=\omega\times(\omega\times\rho)`。

### 3.4 变到 `i` 系

左乘 `A_{0,i}^T=A_{p,i}^TA_{0,p}^T`：

$$
a_i^{\text{线}}
=
A_{p,i}^Ta_p^{\text{线}}
+
A_{p,i}^T\Bigl(
[\omega_p]_\times^2\rho_i
+
[\alpha_p]_\times\rho_i
+
2[\omega_p]_\times\dot\rho_i
+
\ddot\rho_i
\Bigr).
\tag{4.13}
$$

### 3.5 把 `\alpha_p` 和 `\ddot\rho_i` 吸收掉

由 (3.12)，

$$
\bigl(C_{i,p}a_p\bigr)^{\text{线}}
=
A_{p,i}^Ta_p^{\text{线}}
+
A_{p,i}^T[\rho_i]_\times^T\alpha_p.
$$

而 `[\rho]_\times^T\alpha=-\rho\times\alpha=\alpha\times\rho=[\alpha]_\times\rho`，所以

$$
\bigl(C_{i,p}a_p\bigr)^{\text{线}}
=
A_{p,i}^T\bigl(a_p^{\text{线}}+\alpha_p\times\rho_i\bigr).
\tag{4.14}
$$

由 (4.7)–(4.8)，

$$
\bigl(\Phi_i\ddot q_i\bigr)^{\text{线}}
=
A_{p,i}^T\ddot\rho_i.
\tag{4.15}
$$

把 (4.14)、(4.15) 从 (4.13) 中减去，剩下的就是 `\eta_i` 的线性部分：

$$
\boxed{
\eta_i^{\text{线}}
=
A_{p,i}^T\Bigl(
[\omega_p]_\times^2\rho_i
+
2[\omega_p]_\times\dot\rho_i
\Bigr).
}
\tag{4.16}
$$

### 3.6 角速度部分

角速度递推为

$$
\omega_i=A_{p,i}^T\omega_p+\omega_{p,i|i}.
\tag{4.17}
$$

对时间求导：

$$
\frac{d}{dt}\omega_i
=
A_{p,i}^T\alpha_p
+
\bigl(A_{p,i}^T\omega_p\bigr)\times\omega_{p,i|i}
+
\frac{d}{dt}\omega_{p,i|i}.
\tag{4.18}
$$

其中：

- `A_{p,i}^T\alpha_p` 已包含在 `C_{i,p}a_p` 的角速度部分；
- `\frac{d}{dt}\omega_{p,i|i}` 已包含在 `\Phi_i\ddot q_i` 的角速度部分；
- 剩下的是

$$
\boxed{
\eta_i^{\text{角}}
=
\bigl(A_{p,i}^T\omega_p\bigr)\times\omega_{p,i|i}
=
A_{p,i}^T\bigl(\omega_p\times\omega_{p,i|p}\bigr),
}
\tag{4.19}
$$

其中 `\omega_{p,i|p}=A_{p,i}\omega_{p,i|i}` 是相对角速度在 `p` 系中的表示。

### 3.7 合并

$$
\boxed{
a_i
=
C_{i,p}a_p
+
\Phi_i\ddot q_i
+
\eta_i,
}
\tag{4.20}
$$

$$
\boxed{
\eta_i
=
\begin{bmatrix}
A_{p,i}^T\bigl([\omega_p]_\times^2\rho_i+2[\omega_p]_\times\dot\rho_i\bigr)\\[4pt]
A_{p,i}^T\bigl(\omega_p\times\omega_{p,i|p}\bigr)
\end{bmatrix}.
}
\tag{4.21}
$$

这就是讲义 (9.14)–(9.15) 在本模型中的形式。

### 3.8 转动铰/棱柱铰的代入

| 关节类型 | `\rho_i` | `\dot\rho_i` | `\omega_{p,i|p}` |
| :--- | :--- | :--- | :--- |
| 转动铰 | `o_i` | `0` | `n_i\dot q_i` |
| 棱柱铰 | `o_i+n_iq_i` | `n_i\dot q_i` | `0` |

因此：

- 转动铰：`\eta_i^{\text{线}}=A_{p,i}^T[\omega_p]_\times^2o_i`，
  `\eta_i^{\text{角}}=A_{p,i}^T(\omega_p\times n_i\dot q_i)`；
- 棱柱铰：`\eta_i^{\text{线}}=A_{p,i}^T([\omega_p]_\times^2(o_i+n_iq_i)+2[\omega_p]_\times n_i\dot q_i)`，
  `\eta_i^{\text{角}}=0`。

---

## 4. 加速度递推

把 (4.20) 写成从根往下的递推：

$$
\boxed{
a_i=C_{i,p(i)}a_{p(i)}+\Phi_i\ddot q_i+\eta_i,
\qquad
a_0=0.
}
\tag{4.22}
$$

`C_{i,p}` 和 `\Phi_i` 与速度级完全相同；新增的只有 `\eta_i`。

---

## 5. 偏置 `b_i`

定义 `b_i` 为 `\ddot q=0` 时的体加速度：

$$
b_i=C_{i,p(i)}b_{p(i)}+\eta_i,
\qquad
b_0=0.
\tag{4.23}
$$

由递推可得

$$
\boxed{
b_i
=
\sum_{k\in\mathrm{pvec}[i]}C_{i,k}\eta_k.
}
\tag{4.25}
$$

并且

$$
\boxed{
a_i=J_i\ddot q+b_i.
}
\tag{4.26}
$$

证明：把 (4.22) 沿父链展开，

$$
a_i
=
\sum_{k\in\mathrm{pvec}[i]}C_{i,k}\bigl(\Phi_k\ddot q_k+\eta_k\bigr)
=
\Bigl(\sum_{k}C_{i,k}\Phi_k\Bigr)\ddot q
+
\sum_{k}C_{i,k}\eta_k
=
J_i\ddot q+b_i.
$$

这说明 **体加速度对 `\ddot q` 是线性的，而且系数矩阵就是速度级的体雅可比 `J_i`**。
不需要再定义一个“加速度雅可比”。

---

## 6. 点加速度与任务加速度

### 6.1 体上一点

设点 `P` 在体 `i` 上的局部坐标为 `p`：

$$
r_P=r_i+A_{0,i}p.
\tag{4.27}
$$

对时间求二阶导：

$$
\ddot r_P
=
\ddot r_i
+
A_{0,i}\bigl(\omega_i\times\omega_i\times p+\alpha_i\times p\bigr).
\tag{4.28}
$$

左乘 `A_{0,i}^T`：

$$
a_P^{\text{线}}
=
a_i^{\text{线}}
+
\omega_i\times\omega_i\times p
+
\alpha_i\times p.
\tag{4.29}
$$

而

$$
T(p)a_i
=
\begin{bmatrix}
a_i^{\text{线}}-[p]_\times\alpha_i\\
a_i^{\text{角}}
\end{bmatrix}
=
\begin{bmatrix}
a_i^{\text{线}}+\alpha_i\times p\\
a_i^{\text{角}}
\end{bmatrix}.
$$

所以

$$
\boxed{
a_P
=
T(p)a_i
+
\begin{bmatrix}
\omega_i\times\omega_i\times p\\
0
\end{bmatrix}.
}
\tag{4.30}
$$

### 6.2 世界系

任务加速度在世界系中为

$$
\ddot x
=
\begin{bmatrix}
A_{0,i} & 0\\
0 & A_{0,i}
\end{bmatrix}
a_P.
\tag{4.31}
$$

### 6.3 任务偏置

把 (4.26) 代入 (4.30)：

$$
a_P
=
T(p)\bigl(J_i\ddot q+b_i\bigr)
+
\begin{bmatrix}
\omega_i\times\omega_i\times p\\0
\end{bmatrix}.
$$

定义

$$
J_P=T(p)J_i,
\qquad
b_P=T(p)b_i
+
\begin{bmatrix}
\omega_i\times\omega_i\times p\\0
\end{bmatrix}.
\tag{4.32}
$$

则

$$
a_P=J_P\ddot q+b_P.
\tag{4.33}
$$

世界系：

$$
J_T=
\begin{bmatrix}
A_{0,i} & 0\\
0 & A_{0,i}
\end{bmatrix}J_P,
\qquad
b_T=
\begin{bmatrix}
A_{0,i} & 0\\
0 & A_{0,i}
\end{bmatrix}b_P.
\tag{4.34}
$$

于是

$$
\boxed{
\ddot x=J_T\ddot q+b_T.
}
\tag{4.35}
$$

默认 `i=13`、`p=0` 时，

$$
J_T=
\begin{bmatrix}
A_{0,13} & 0\\
0 & A_{0,13}
\end{bmatrix}J_{13},
\qquad
b_T=
\begin{bmatrix}
A_{0,13} & 0\\
0 & A_{0,13}
\end{bmatrix}b_{13}.
\tag{4.36}
$$

---

## 7. 数值校核

`verify_kinematics.py` 在随机位形（`seed=20261122`）下，用中心差分验证加速度：

$$
q(t\pm h)=q\pm h\dot q+\frac{h^2}{2}\ddot q,
\qquad
\dot q(t\pm h)=\dot q\pm h\ddot q.
$$

然后比较

$$
\frac{A_{0,i}(t+h)v_i(t+h)-A_{0,i}(t-h)v_i(t-h)}{2h}
\quad\text{vs}\quad
A_{0,i}(t)a_i(t).
$$

结果：

| 检查项 | 最大误差 | 容差 |
| :--- | ---: | ---: |
| 体加速度 vs 世界速度差分 | `4.119e-10` | `1e-7` |
| 闭环 `c_ddot` vs 位置二阶差分 | `1.641e-08` | `1e-6` |

第一项验证了 `a_i` 的定义、`\eta_i` 的符号和 `C_{i,p}a_p` 的吸收；
第二项验证了第 5 章的闭环加速度公式。

---

## 8. 本章小结

- 加速度定义为 `a=[A^T\ddot r;\dot\omega]`，不是 `d/dt v`。
- 转动铰和棱柱铰的相对加速度都是 `\Phi_i\ddot q_i`，`\xi_i=0`。
- `\eta_i` 由离心项、科氏项和角速度叉乘项组成；`\alpha_p\times\rho` 已被 `C_{i,p}a_p` 吸收。
- 加速度递推 `a_i=C a_p+\Phi\ddot q+\eta_i`。
- 偏置 `b_i=\sum C_{i,k}\eta_k`，且 `a_i=J_i\ddot q+b_i`；加速度的系数矩阵就是速度雅可比。
- 点/任务加速度 `a_P=T(p)a_i+[\omega\times\omega\times p;0]`，
  `\ddot x=J_T\ddot q+b_T`。
