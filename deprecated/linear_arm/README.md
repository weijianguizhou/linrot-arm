# 直线模组手臂 URDF

拓扑按 `手臂动力学拓扑图.png`（B0..B12，R = 转动副，L = 棱柱副）；
几何 / 质量 / 惯量 / 关节站位来自 SolidWorks 装配 `手臂改10.10.SLDASM`。

```
urdf/linear_arm.urdf    13 个 link、12 个关节（9 转动 + 3 棱柱），树结构
meshes/B0..B12.stl      每个刚体的合并网格（真实 CAD 网格，mm）
loops.yaml              被剪掉的 3 条闭环边（URDF 表达不了环）
mapping.json            每个 B_i 由哪些 CAD 零件组成 + 质量 + 站位
urdf_check.png          零位渲染（X-Z / Y-Z），用来一眼核对
tools_build.py          生成器（改表即可重出）
tools_check.py          校验 + 渲染
```

## 怎么用

```bash
# 校验（XML / 树 / 质量）
python tools_check.py

# 改了 mapping 或站位后重出
python tools_build.py
```

导入 MuJoCo 时注意：URDF 里 mesh 用相对路径，`<mesh filename="meshes/B1.stl"/>`。

## 数据来源与可信度

| 项 | 来源 | 可信度 |
| --- | --- | --- |
| link 质量 / 质心 / 惯量张量 | SolidWorks `GetMassProperties2`（SI，CAD 材质密度） | 高 |
| link 网格 | 逐零件导出 STL，按装配变换摆位后合并 | 高 |
| R4 / L1 站位与轴 | `linear_4000_up_trans` / `linear_3000_down_trans` 坐标系 + 模组轴线 | 高 |
| R8 / L2 站位与轴 | `linear_400_r1_trans` / `r2_trans` | 高 |
| R9 / L3 站位与轴 | `linear_400_l1_trans` / `l2_trans` | 高 |
| R11 站位与轴 | `joint5_trans`（轴 = 该坐标系 Z） | 高 |
| R12 站位与轴 | `joint6_trans` | 高 |
| R1 轴 | `base_link` 坐标系 Z（竖直回转） | 中 |
| R5 | `part4_肘部连接件` 质心 + 沿 Y 轴 | 中 |
| R6 | `linear_4000_trans` 中段 + 沿 Y 轴 | 中 |
| R2 / R3 | 两台 EC-A6013 电机位置推断 + 沿 Y 轴 | **低 —— 需确认** |
| R7 / R10 | 十字轴 / 腕部连接件位置推断 + 沿 Y 轴 | **低 —— 需确认** |
| 各关节 limit（effort/velocity/行程） | 占位值，CAD 里没有 | **占位，必须替换** |
| B0 质量 | 基座法兰是"虚拟零件"，SolidWorks 给不出质量属性 | **缺，按 0 写** |

CAD 里作者预留了 12 个命名坐标系 + 12 条基准轴（`base_link`、`joint3/5/6/7`、
`linear_4000(_up/_down)`、`linear_400_r/l(_1/_2)`），上面"高"信度的站位就是从它们直接读的。
`joint3` / `joint7` 只有基准轴、没有坐标系，所以只能推断。

## 闭环（URDF 只能是树）

图里有 3 个独立环：15 条边 − 13 个节点 + 1 = 3。本 URDF 按下面方式各断一条，
断点写进 `loops.yaml`（含站位与轴向），在 MuJoCo / SDF 里用
`<equality><connect>` 或 `<joint>` 补回来：

| 环 | 组成 | 断掉的边 | 理由 |
| --- | --- | --- | --- |
| 1 | B3-R4-B4-L1-B5-R5-B6-R6-B3 | **R6** | 保留丝杠驱动链 R4→L1→R5，R6 是被约束的冗余铰 |
| 2 | B6-R7-B7-R10-B12-R11-B10-L2-B8-R8-B6 | **R10** | B7 保留为末端摆杆 |
| 3 | B6-R9-B9-L3-B11-R12-B12 | **R12** | B11 保留为末端摆杆 |

换个断法只需改 `tools_build.py` 里的 `CUTS` 再跑一次——三种断法都能得到合法的树。

## CAD 本身的两个坑（不是转换问题，是模型状态）

1. **`dg4000`（1.65 kg 的大模组）挂在 `肘部` 子装配里，位置在 x≈−470 mm 外**，
   明显是搁在旁边的参照件，没装进手臂链路。手臂上实际用的是
   `ceshi_4000_link` + `zhoubushizijia` + `taotong1/2` 这套"测试"模组。
2. **两只 `ceshilinear400` 质量只有 1.6 g**，是空壳占位件（腕部真正的执行器是
   `腕部2`，各 0.54 kg）。所以 B8/B9 的质量是按 `腕部2` 算的，
   `ceshilinear400` 的网格只是外形。

如果这两处补成真实件，B4 / B8 / B9 的质量和惯量会变，重跑 `tools_build.py` 即可。

## B_i ↔ CAD 零件

见 `mapping.json`。摘要：

| link | 质量 (kg) | 主要零件 |
| --- | --- | --- |
| B0 | 0（缺） | body固定法兰(新) ×2、part1连接法兰（新）×2 |
| B1 | 0.705 | EC-A6013 电机 #1、part1连接法兰2（新） |
| B2 | 0.704 | EC-A6013 电机 #2、part3 法兰3（新） |
| B3 | 0.954 | part4_肘部主支架、part4_Joint3-Joint4_法兰1/2、part3 法兰2、m6-10 |
| B4 | 0.061 | 测试丝杠模组：ceshi_4000_link、new_linear4000_link2、zhoubushizijia、taotong1/2、626Z 轴承 ×2 |
| B5 | 0.149 | part4_肘部连接件、鱼眼轴承 |
| B6 | 0.499 | part56_腕部主支架 |
| B7 | 0.014 | part56_腕部十字轴 |
| B8 | 0.542 | 腕部2-1、ceshilinear400-1 |
| B9 | 0.542 | 腕部2-3、ceshilinear400-2 |
| B10 | 0.021 | 腕部-1、626Z-5 |
| B11 | 0.021 | 腕部-3、626Z-6 |
| B12 | 0.100 | part56_腕部连接件、m4-8 |
| **合计** | **4.312** | |

## 约定

所有 link 坐标系**与世界系平行**，原点放在其父关节的站位点上，所以：

* `<joint><origin>` 只有平移，`rpy` 全 0；
* `<inertial><origin>` 是质心相对 link 原点的平移；
* 惯量张量已经是世界姿态下的分量（因为 link 姿态 = 世界姿态，不用再转）。

这样改一个站位 = 改一列数字，不会牵扯旋转。

## 重新生成的依赖

`tools_build.py` 读的是这次从 CAD 抽出来的中间数据（都在仓库里，别删）：

```
D:\robotics\_research\frames.json    装配里 12 个命名坐标系的位姿
D:\robotics\_research\world.json     53 个组件的世界变换 + 质量属性
D:\robotics\_research\stl_map.json   组件 -> STL 文件名
D:\robotics\_research\stl\           逐零件原始 STL（22 MB）
```

这些是 SolidWorks COM 一次性抽出来的（脚本在 `_research/` 里：
`explore_asm.py` / `export_stl.py` / `export_stl2.py` / `extract_frames.py`）。
改了 CAD 就重跑抽取，再 `python tools_build.py`。
