# 直线模组手臂 · 动力学模型（q=0 竖直标准位）

本目录是**唯一**与 `..\docs\手臂动力学模型.png` 一致的 URDF / MJCF 模型。
12 刚体（`B0..B11`）、14 关节（11 个树关节 + 3 个闭环关节），零位 `q=0` 为参考图中的竖直伸展姿态。

```
model/
├─ urdf/linear_arm.urdf      机器人模型（米，12 link / 11 树关节）
├─ mjcf/linear_arm.xml       MuJoCo 版本（含 fullinertia + 3 个 equality/connect 闭环）
├─ loops.yaml                3 个闭环关节的锚点与轴向（JSON 也是合法 YAML）
├─ mapping.json              每个刚体的零件构成、质量、质心、站位
├─ meshes/                   B0.stl .. B11.stl（单位：米）
├─ meshes_mm_backup/         同上，单位：毫米（存档用）
├─ preview/
│   ├─ p_zero_4view.png      零位四视图
│   ├─ compare_photo.png     参考图 vs 模型
│   ├─ labeled_B0_B11.png    刚体编号标注
│   └─ _archive/             历史调试图
├─ tools_build_v2_q0.py      由 CAD 导出数据重建整套模型
├─ tools_verify.py           总校验（加载/尺度/质量/闭环/惯量）
├─ tools_check_deep.py       深度校验（闭环锚点残差、关节站位包络、质心落在网格内）
└─ _legacy/                  旧的错误版本生成脚本（已废弃，勿再运行）
```

> 打开界面看模型、MuJoCo 内动力学校验的脚本在 `..\sim\`（`view.py`、`check_model.py`）。

---

## 1. 坐标约定（看模型前必须先读这一段）

| 项目 | 约定 |
| :--- | :--- |
| URDF 根 `base_link` / `B0` 坐标系 | 位于 **B0 的站位**，不是 CAD 全局原点 |
| CAD 坐标 → URDF 坐标 | `URDF = CAD - B0_origin`，其中 `B0_origin = [-88.898, 78.26, 33.11] mm` |
| 长度单位 | `urdf/linear_arm.urdf` 与 `meshes/*.stl` 均为 **米**（`meshes/.in_meters` 为哨兵文件） |
| link 姿态 | 所有 link 在 `q=0` 时与世界系平行（`rpy` 全为 0），关节 `origin` 只是相对父帧的平移 |
| 网格原点 | 每个 `B{i}.stl` 已平移到对应 link 坐标系，`<mesh>` 不再需要 `scale` 或 `origin` 修正 |
| 关节轴朝向 | `R5/R9/R11` 在 `loops.yaml` 里记为 `[0,-1,0]`，这是**父件视角**的轴向；URDF 树关节统一用 `[0,1,0]` 等正方向 |

## 2. 刚体构成

`B0..B11` 对应参考图中的 `B0..B12`：第 3 肩关节与上臂主支架合并为 `B2`，其后全部顺延一位。

| 刚体 | 参考图 | 包含零件 | 质量 (kg) |
| :--- | :--- | :--- | ---: |
| `base_link` | — | `body固定法兰(新）` ×2（上下抱箍，夹住 B0 电机） | — |
| `B0` | `B0` | `EC-A6013-H20-100-1`、`part1连接法兰（新）-1` | 0.6400 |
| `B1` | `B1` | `part1连接法兰2（新）-1`、`EC-A6013-…-2`、`part1连接法兰（新）-2`、`part3 法兰2（新）-1`、`part3 法兰3（新）-2`、`EC-A3814-H14-107-1` | 1.4029 |
| `B2` | `B2 + B3` | `part4_Joint3-Joint4_法兰1-1`、`…法兰2_new2-1`、`part4_肘部主支架 - 改-1`、`zhoubushizijia-1`、`taotong1-1/3`、`Rolamento 626Z-2/3`（肘轴承，z=-381.4 mm） | 0.7659 |
| `B3` | `B4` | `dg4000-1`、`改new_linear4000_link-1` | 1.6555 |
| `B4` | `B5` | `肘部连接1-1`、`鱼眼轴承-3`、`改new_linear4000_link2-1`、`taotong2-2/3` | 0.0249 |
| `B5` | `B6` | `part4_肘部连接件-1`、`part56_腕部主支架 -改-1`、`Rolamento 626Z-5/6`（腕十字轴轴承，z=-651.9 mm） | 0.6604 |
| `B6` | `B7` | `part56_腕部十字轴-1`（尾轴向后伸出，即参考图 `B7` 箭头所指） | 0.0135 |
| `B7` / `B8` | `B8` / `B9` | `腕部2-1` / `腕部2-3` | 0.5400 / 0.5400 |
| `B9` / `B10` | `B10` / `B11` | `腕部-1` / `腕部-3` | 0.0098 / 0.0098 |
| `B11` | `B12` | `part56_腕部连接件-1` | 0.0996 |
| **合计** | | | **6.3624** |

## 3. 关节

### 3.1 树关节（URDF `<joint>`，11 个）

| 关节 | 类型 | 父 → 子 | `origin` (m，相对父帧) | 轴 |
| :--- | :--- | :--- | :--- | :--- |
| `R1` | revolute | `B0 → B1` | `[0, 0, 0]` | `[0, 1, 0]` |
| `R2` | revolute | `B1 → B2` | `[0, 0.05875, -0.104612]` | `[0, 0, 1]` |
| `R3` | revolute | `B2 → B3` | `[0, 0, -0.0623883]` | `[0, 1, 0]` |
| `L1` | prismatic | `B3 → B4` | `[-0.00617502, 0, -0.224006]` | `[-0.0275558, 0, -0.99962]` |
| `R4` | revolute | `B4 → B5` | `[-0.00132494, 0, -0.0480638]` | `[0, 1, 0]` |
| `R6` | revolute | `B5 → B6` | `[0.057282, 0, -0.24743]` | `[-1, 0, 0]` |
| `R7` | revolute | `B5 → B7` | `[0.0205, 0.036, -0.08243]` | `[0, 1, 0]` |
| `R8` | revolute | `B5 → B8` | `[0.0205, -0.036, -0.08243]` | `[0, 1, 0]` |
| `L2` | prismatic | `B7 → B9` | `[-0.00958239, 0, -0.119617]` | `[-0.0798533, 0, -0.996807]` |
| `R10` | revolute | `B9 → B11` | `[-0.00363561, 0, -0.0453832]` | `[0, 1, 0]` |
| `L3` | prismatic | `B8 → B10` | `[-0.00958239, 0, -0.119617]` | `[-0.0798533, 0, -0.996807]` |

`R3–L1–R4` 与闭环 `R5` 构成肘部四连杆；`R7–L2–R10`、`R8–L3–R11` 与闭环 `R9` 构成腕部 2-DOF 并联机构。

### 3.2 闭环关节（URDF 树里断开，用等式约束补回，3 个）

| 关节 | 类型 | 父 → 子 | 站位（URDF 根系，mm） | 轴 |
| :--- | :--- | :--- | :--- | :--- |
| `R5` | revolute | `B2 → B5` | `[13.0, 58.75, -414.5]` | `[0, -1, 0]` |
| `R9` | revolute | `B6 → B11` | `[39.78, 58.75, -686.5]` | `[0, -1, 0]` |
| `R11` | revolute | `B10 → B11` | `[-0.22, 22.75, -686.5]` | `[0, -1, 0]` |

- `loops.yaml` 记录了 CAD 绝对坐标 `origin_mm` 以及父/子件局部锚点 `origin_parent_mm` / `origin_child_mm`。
- `mjcf/linear_arm.xml` 用 `<equality><connect>` 实现，`anchor` 取父件局部锚点。
- **`q=0` 时三个闭环的锚点残差均为 0.000 mm**，MuJoCo 等式约束残差为 `[0, 0, 0]`。

## 4. 使用

**在 MuJoCo 界面里看模型（推荐从这一步开始）**

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"
cd D:\robotics\linerrobot
& $PY -X utf8 .\sim\view.py            # 关重力打开，手臂不会塌，可用鼠标摆姿态
& $PY -X utf8 .\sim\view.py --gravity  # 真实重力，看它会怎么垂下去
```

界面操作：左键旋转、右键平移、滚轮缩放、空格暂停、`Ctrl + 右键拖动` 直接拖刚体摆姿态；左侧 `Physics` 面板可以随时改重力和时间步。

也可以用官方启动器（等价，但重力还是 9.81，模型没有电机，一打开就会垂下去）：

```powershell
& $PY -m mujoco.viewer --mjcf="D:\robotics\linerrobot\model\mjcf\linear_arm.xml"
```

**在代码里加载**

```python
import mujoco
m = mujoco.MjModel.from_xml_path(r"D:\robotics\linerrobot\model\mjcf\linear_arm.xml")
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
```

URDF 也可直接加载（`mujoco.MjModel.from_xml_path(urdf)`），但此时 3 个闭环关节丢失，只保留开链。

## 5. 复现与校验

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"
cd D:\robotics\linerrobot\model
& $PY -X utf8 .\tools_build_v2_q0.py    # 由 ..\build\data\world_raw.json + stl 重建整套模型
& $PY -X utf8 .\tools_verify.py         # 应输出「总判定: 通过」
& $PY -X utf8 .\tools_check_deep.py     # 应输出「问题汇总: 无」
& $PY -X utf8 .\tools_preview.py        # 重出 preview/ 下的图
```

当前校验结论：

- URDF `nbody=12, njnt=11`；MJCF `nbody=13, njnt=11, neq=3`
- 整机包围盒 `137 × 202 × 767 mm`
- URDF Σmass = mapping Σmass = `6.3624 kg`（MuJoCo 里 B0 被 `fusestatic` 并入 world，故 body 合计 5.7224 kg）
- 12 个刚体的惯量全部正定并满足三角不等式；所有质心都落在自身网格包围盒内
- 闭环锚点残差 `0.000 mm`

## 6. 相比旧版本改了什么

旧版（`_legacy/`）之所以和参考图对不上，有三个根因：

1. **SolidWorks 旋转矩阵被转置**。`IMathTransform.ArrayData` 的前 9 个数是局部基向量在装配体坐标系下的分量（变换矩阵的**列**），旧脚本按**行**填进矩阵，于是 `world_raw.json` 里存的实际是 `R^T`。
   - 后果一：腕部两只电缸 `腕部2-1/3` 被画成水平横躺（真实 CAD 里是竖直的），推杆也跟着错位。
   - 后果二：`dg4000` 被误判为「飘在侧面」，于是旧脚本用包围盒强行吸附，把它压低了 100 多毫米，还漏掉了顶部的 `改new_linear4000_link-1` 和底部的 `肘部连接1-1`。
2. **网格偏移用错了质心**。STL 被 SolidWorks 导出到第一卦限，需要平移回零件局部原点。旧脚本用表面面积形心 `trimesh.centroid`，正确做法是体积质心 `trimesh.center_mass`，非对称件上最多差 15 mm。
3. **肩部漏件 + 二次拼装**。旧版漏掉了第二个 `EC-A6013` 外壳和 `part1连接法兰（新）`，肩部比参考图少一个电机鼓包；后来的「photo 版」又把腕部电缸挂到小臂支架底部以下悬空，「v3 版」干脆抛弃 CAD 配合点做包围盒堆叠，还把毫米网格配米制关节坐标。

## 7. 已知未完成项

- 关节没有写 `<limit>`（参考图与 CAD 都没有给出行程/角度范围），需要按实际电缸行程补：`L1`、`L2`、`L3` 为直线行程，`R1..R11` 为角度范围。
- 没有做碰撞体简化，`collision` 直接复用高精度视觉网格（面片数多，适合静力学/动力学验证，不适合大规模并行仿真）。
- `D:\robotics\_research\rebuild_world_raw.py` 仍然按转置矩阵写 `world_raw.json`；本目录所有工具都用 `R_true = M[:3,:3].T` 还原，二者约定一致。若要改 `rebuild_world_raw.py`，必须同步改 `tools_build_v2_q0.py`。
