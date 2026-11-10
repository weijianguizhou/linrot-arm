# linrot-arm

直线电缸 + 旋转电机混合驱动的六自由度机械臂。名字取自 **lin**ear + **rot**ary。

它不是普通串联臂：肘部是"电缸 + 四连杆"闭环，腕部是"双电缸 + 十字轴"并联闭环。
所以建模时必须处理 **3 个闭环约束**，不能当成开链来算。

**当前用途**：样机控制器的离线验证。在真机上线之前，把笛卡尔空间位置控制、电缸行程、
关节限位、奇异位形都在仿真里跑一遍。

---

## 1. 机构

| 部位 | 自由度 | 执行器 | 传动 |
| :--- | ---: | :--- | :--- |
| 肩 | 3 | `EC-A6013` ×2、`EC-A3814` ×1 | 电机直连法兰，三轴正交 |
| 肘 | 1 | `dg4000` 直线电缸 | 四连杆闭环（`R6`） |
| 腕 | 2 | 腕部直线电缸 ×2 | 十字轴 + 双电缸并联（`R10`、`R12`） |

**有效自由度 = 6**：12 个树关节 − 3 个闭环约束 = 6，已用闭环约束 Jacobian 求秩验证。

刚体划分 13 个（`B0..B12`），与 `docs/手臂动力学模型.png` 的标号完全对应。
总质量 **6.3624 kg**（CAD 真值），惯量用 CAD 完整张量（含非对角项）。

肩部三条轴在 CAD 里并不是正放的：

| 轴 | 方向 | 说明 |
| :--- | :--- | :--- |
| 肩 1 | `[0.130526, 0.991445, 0]` | 整个电机座绕 Z 偏了 7.495° |
| 肩 2 | `[0.991445, -0.130526, 0]` | 同上，与之正交 |
| 肩 3 | `[0, 0, 1]` | `EC-A3814`，竖直，与配合 `同心532` 一致 |

## 2. 目录

```
linrot-arm/
├─ README.md            本文件：项目说明、进度、待办
├─ AGENTS.md            给 AI 助手的交接说明（新会话必读）
├─ model/               URDF / MJCF / 网格 / 重建与校验工具   ← 主要工作区
│  ├─ urdf/linear_arm.urdf
│  ├─ mjcf/linear_arm.xml
│  ├─ meshes/           B0..B12.stl（米制）
│  ├─ loops.yaml        3 个闭环关节的锚点与轴向
│  ├─ mapping.json      每个刚体的零件构成、质量、质心、站位
│  ├─ preview/          预览图（四视图、参考图对比、B0..B12 标注）
│  ├─ tools_build_13body.py   由 CAD 数据重建整套模型
│  ├─ tools_verify.py         总校验
│  ├─ tools_check_deep.py     深度校验
│  └─ tools_preview.py        重出预览图
├─ sim/                 view.py（开界面）、check_model.py（9 节动力学校验）
├─ build/data/          world_raw.json、mate_graph.json、frames.json、stl_map.json、stl/
├─ cad/                 SolidWorks 源件（3 个 SLDASM + 24 个 SLDPRT）
└─ docs/                手臂动力学模型.png/.vsdx、手臂动力学拓扑图.png/.vsdx、零件归属确认表.xlsx
```

## 3. 快速开始

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"
cd D:\robotics\linrot-arm
& $PY -X utf8 .\sim\view.py          # 打开 MuJoCo 界面
& $PY -X utf8 .\sim\check_model.py   # 9 节动力学校验
```

`view.py` 默认把重力设成 0——模型还没有 actuator，开着重力会直接垂下去。
想看重力效果加 `--gravity`。

WSL 下同样可用（`mujoco 3.16.0` 与 Windows `3.14.0` 数值逐项一致）：

```bash
python sim/check_model.py
```

## 4. 当前进度

### 已完成

- **13 刚体模型**：`B0..B12`，CAD 真实质量、质心、惯量张量。
- **12 树关节 + 3 闭环约束**：`R6`（肘四连杆）、`R10`（腕十字轴第二轴）、`R12`（腕左推杆），
  `q=0` 时闭环残差 **0.0004 mm**。
- **肩部三轴定标**：用电机网格做最小二乘圆拟合，再用"电机→法兰共轴链"验证
  （垂直偏差 0.008 mm / 0.039 mm），修正了原先 7.5° 的轴向错误和 40 mm 以上的轴点偏移。
- **零位姿态**：`q=0` 为参考图的竖直伸展姿态，13 刚体 FK 对 CAD 最大偏差 **0.013 mm**。
- **可复现构建**：`model/tools_build_13body.py` 能从 `build/data/` 从头重建
  网格、URDF、MJCF、`loops.yaml`、`mapping.json`。
- **三套校验脚本**全部通过：

  ```
  model\tools_verify.py      总判定: 通过     （nbody=13, njnt=12, neq=3, 包围盒 137×202×767 mm）
  model\tools_check_deep.py  问题汇总: 无     （闭环锚点残差 0，惯量正定）
  sim\check_model.py         全部通过          （有效自由度 6，FK 0.013 mm）
  ```

- **Windows 与 WSL 双端验证**：MuJoCo 3.14.0 / 3.16.0 输出完全一致。

### 提交历史

```
ec5332c  拆分 B2 补回第三个肩轴，改成 13 刚体 B0..B12
f8bb391  仓库改名为 linrot-arm，并去掉建模数据里的绝对路径
f9e7208  初始导入（12 刚体版）
```

## 5. 待办

### 待外部确认

- 电机控制组：6 个执行器接受的指令类型（位置 / 速度 / 力矩）。
- 型号手册参数：`EC-A6013`、`EC-A3814` 的角度范围与减速比；`dg4000` 与腕部电缸的行程和额定推力。

### P0 · 模型

- **关节限位**：写进 URDF `<limit>` 与 MJCF `range`。现在完全没写，MuJoCo 里是无界关节。
- **`armature`**：质量矩阵条件数 3.1e6，某些方向惯量极小（6.2e-07 kg·m²），
  不加会在仿真里数值发飘。
- **`actuator`**：位置 / 速度 / 力矩三套参数并存，等指令类型确定后启用其中一套。

### P1 · 仿真

- **`tool0` 腕心坐标系** + 可配置偏置（控制器以腕心为末端控制点）。
- **碰撞体**：现在 `contype=0`，完全不做碰撞检测。需要给 13 个刚体各做简化碰撞体。
- **IK 与工作空间 / 奇异位形分析**：控制器是笛卡尔空间位置控制，这些是验证的前提。

### P2 · ROS 2

- 接口形式（倾向 `ros2_control` 的 `SystemInterface` 插件）。
- 关节命名是否从 `R1..R12` 改成 `joint1..joint6`。

## 6. 设计决策

| 项目 | 决定 |
| :--- | :--- |
| 刚体划分 | 13 刚体 `B0..B12`，与参考图标号一一对应 |
| 零位姿态 | `q=0` 为竖直伸展姿态（参考图姿态） |
| 控制层级 | 笛卡尔空间位置控制，控制器内部解 IK |
| 末端控制点 | 腕心（球腕中心），叠加可配置偏置 |
| 限位与行程 | 从型号手册取值后写入 URDF / MJCF |
| 控制器对接 | ROS 2 节点 |
| 验收标准 | 先跑通，后续边学边优化 |

## 7. 仓库与大文件

网格与 CAD 走 Git LFS，代码与元数据是普通文件。

```bash
git lfs install
git clone https://github.com/weijianguizhou/linrot-arm.git
cd linrot-arm && git lfs pull
```

**在 WSL 里 clone 之后一定要执行一次**，否则 `git push` 会报
`Remote "origin" does not support the Git LFS locking API`：

```bash
git config lfs.https://github.com/weijianguizhou/linrot-arm.git/info/lfs.locksverify false
```

本机代理环境下，WSL 里 git 需要显式走代理（`127.0.0.1:7890`，需把客户端切到全局模式，
规则模式下 github.com 会被判为 DIRECT 而连接失败）：

```bash
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890
```

## 8. 已知的坑

- **`world_raw.json` 里存的是转置矩阵**。SolidWorks `IMathTransform.ArrayData` 的前 9 个数
  是局部基向量在装配体系下的分量（变换矩阵的**列**），导出脚本按**行**填了，
  所以 `world[:3,:3]` 实际是 `R^T`。取真实旋转必须 `R_true = world[:3,:3].T`。
- **网格是按 CAD 世界姿态烘焙的**，只做了平移。所以 `q=0` 时所有 body 帧同向，
  关节 `rpy` 全为 0，轴向量必须写世界方向。
- **STL 被导到第一卦限**，要用 `trimesh.center_mass`（体积质心）算平移量，
  不能用 `trimesh.centroid`（面积加权，非对称件会差 15 mm）。
- **`build/data/stl_map.json` 用的是纯文件名**，不要写回绝对路径，否则 WSL 下会失效。
