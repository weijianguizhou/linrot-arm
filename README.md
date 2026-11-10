# linrot-arm

直线电缸 + 旋转电机混合驱动的六自由度机械臂。肘部是四连杆闭环，腕部是双电缸并联闭环，
所以既不是普通串联臂，也不是纯直线机构 —— 名字取自 **lin**ear + **rot**ary。

目标用途：**样机控制器的离线验证**。在真机上线之前，把笛卡尔空间位置控制、电缸行程、
关节限位、奇异位形都在仿真里跑一遍。

## 目录

```
linrot-arm/
├─ model/         URDF / MJCF / 网格 / 重建与校验工具   ← 主要工作区
├─ sim/           view.py（打开 MuJoCo 界面）、check_model.py（动力学校验）、adams/
├─ build/data/    world_raw.json、mate_graph.json、frames.json、stl_map.json、stl/
├─ cad/           SolidWorks 源件（3 个 SLDASM + 24 个 SLDPRT）
├─ docs/          手臂动力学模型.png/.vsdx、手臂动力学拓扑图.png/.vsdx、零件归属确认表.xlsx
└─ deprecated/    三个早期的错误 URDF 版本（12 刚体之前的历史）
```

## 机构

| 部位 | 自由度 | 执行器 | 传动 |
| :--- | :--- | :--- | :--- |
| 肩 | 3 | `EC-A6013` ×2、`EC-A3814` ×1 | 电机直连法兰 |
| 肘 | 1 | `dg4000` 直线电缸 | 四连杆（闭环 `R6`） |
| 腕 | 2 | 腕部直线电缸 ×2 | 十字轴 + 双电缸并联（闭环 `R10`/`R12`） |

**有效自由度 = 6**，已用闭环约束 Jacobian 求出秩为 6 验证（12 个树关节 − 3 个闭环约束 = 6）。

## 快速开始

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"
cd D:\robotics\linrot-arm
& $PY -X utf8 .\sim\view.py          # 打开 MuJoCo 界面（默认关重力，可用鼠标拖）
& $PY -X utf8 .\sim\check_model.py   # 9 节动力学校验，末行给结论
```

`view.py` 默认把重力设成 0，因为模型还没有 actuator，开重力会直接垂下去。
想看重力效果就加 `--gravity`。

WSL 下同样可用（`mujoco 3.16.0` 与 Windows `3.14.0` 数值逐项一致）：

```bash
python sim/check_model.py
```

## 仓库与大文件

网格与 CAD 走 Git LFS（`.gitattributes` 已配置），代码与元数据是普通文件。

```bash
git lfs install
git clone https://github.com/weijianguizhou/linrot-arm.git
cd linrot-arm && git lfs pull
```

## 已确定的设计决策

| 项目 | 决定 |
| :--- | :--- |
| 刚体划分 | 13 刚体 `B0..B12`，与 `docs/手臂动力学模型.png` 的标号完全对应 |
| 零位姿态 | `q=0` 为参考图的竖直伸展姿态 |
| 执行器指令类型 | 待与电机控制组确认（位置 / 速度 / 力矩），`sim/` 里先做成可切换 |
| 控制层级 | 笛卡尔空间位置控制，控制器内部解 IK |
| 末端控制点 | 腕心（球腕中心），叠加可配置偏置 |
| 限位与行程 | 从型号手册取值后写入 URDF `<limit>` / MJCF `range` |
| 控制器对接 | ROS 2 节点（倾向 `ros2_control` 的 `SystemInterface`） |
| 验收标准 | 先跑通用，后续边学边优化 |

## 未完成项

**待外部确认**

- 电机控制组的指令接口（位置 / 速度 / 力矩）。
- 型号手册参数：`EC-A6013`、`EC-A3814` 的角度范围与减速比；`dg4000` 与腕部电缸的行程和额定推力。

**P0 · 模型**

- 关节限位写进 URDF `<limit>` 与 MJCF `range`。
- 加 `armature`：质量矩阵条件数 3.1e6，某些方向惯量极小，不加会在仿真里飘。
- 加 `actuator`（位置 / 速度 / 力矩三套参数并存，等指令类型确定后启用其中一套）。

**P1 · 仿真**

- `tool0` 腕心坐标系 + 可配置偏置。
- 碰撞体：现在 `contype=0` 完全不做碰撞检测，需要给 13 个刚体各做简化碰撞体。
- IK 与工作空间 / 奇异位形分析。

**P2 · ROS 2**

- 接口形式定案（倾向 `ros2_control` 的 `SystemInterface` 插件）。
- 关节命名是否从 `R1..R12` 改成 `joint1..joint6`。
- WSL 侧 `mujoco 3.16.0` 已验证可用，无需降版本。

## 当前状态

模型侧已完成：13 刚体、12 树关节 + 3 闭环、CAD 真实质量与惯量、`q=0` 闭环残差 0。

三项校验均可复现通过：

```
model\tools_verify.py      总判定: 通过
model\tools_check_deep.py  问题汇总: 无
sim\check_model.py         全部通过（有效自由度 6）
```
