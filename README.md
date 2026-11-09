# linerrobot · 线性电缸手臂

一台用**直线电缸 + 连杆闭环**驱动的 3+1+2 自由度机械臂。本目录集中了它的全部资产：SolidWorks 源文件、URDF/MJCF 模型、建模数据、仿真与控制脚本。

```
linerrobot/
├─ model/        机器人模型（URDF / MJCF / 网格 / 校验工具）  ← 主要入口
├─ sim/          MuJoCo 查看与仿真（view.py / check_model.py）、ADAMS 导出
├─ build/data/   建模原始数据（world_raw.json / stl_map.json / mate_graph.json / frames.json / stl/）
├─ cad/          SolidWorks 源装配与零件（.SLDASM / .SLDPRT）
├─ docs/         参考图（手臂动力学模型 / 拓扑图）、零件归属确认表
└─ deprecated/   三个已废弃的历史 URDF 版本
```

## 这是什么

| 部分 | 自由度 | 机构 |
| :--- | :--- | :--- |
| 肩 | 3 | 两个 `EC-A6013` 正交布置 + 一个 `EC-A3814` 竖直轴 |
| 肘 | 1 | `dg4000` 直线电缸 + 四连杆闭环（`R3–L1–R4` 与 `R5`） |
| 腕 | 2 | 两只 `腕部2` 直线电缸 + 十字轴并联闭环 |

零位 `q=0` 是参考图中的竖直伸展姿态；`mjcf/linear_arm.xml` 用 `<equality><connect>` 把三个闭环关节约束回来，`q=0` 时残差 0.000 mm。

## 快速开始

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"
cd D:\robotics\linerrobot
& $PY -X utf8 .\sim\view.py            # 打开 MuJoCo 界面（默认关重力，可用鼠标拖着摆姿态）
& $PY -X utf8 .\sim\check_model.py     # 9 节动力学校验
```

模型细节、坐标约定、关节表、重建方法见 `model/README.md`。

## 仓库与大文件

仓库根就是这个目录。网格和 CAD 源文件走 **Git LFS**（见 `.gitattributes`）：

| 类型 | 处理 |
| :--- | :--- |
| `*.stl` / `*.STL`、`*.SLDASM`、`*.SLDPRT`、`*.STEP`、`*.vsdx`、`*.xlsx`、`*.JPG` | Git LFS |
| `.py`、`.urdf`、`.xml`、`.json`、`.yaml`、`.md` | 普通提交 |

新机器上第一次用：

```bash
git lfs install
git clone <repo-url>
cd linerrobot
git lfs pull          # 若 clone 时没自动拉 LFS 对象
```

## 已确定的设计决策

| 项 | 决定 |
| :--- | :--- |
| 项目目标 | **样机控制器的离线验证**（不是纯演示，也不是论文数值） |
| 刚体编号 | **13 刚体 `B0..B12`**，与 `docs/手臂动力学模型.png` 完全对应（当前模型还是 12 刚体，待拆） |
| 肩部轴数 | **3 个**独立可控轴（当前模型只有 2 个，待补） |
| 控制架构 | **笛卡尔空间位置控制**（上位机给末端位姿，控制器内部解 IK） |
| 末端定义 | **腕心**（十字轴交点），并叠加可配置的工具偏置，默认 0 |
| ROS 对接 | **ROS 2 节点**，仿真扮演硬件；真机控制器代码不改 |
| 运行方式 | Windows 上整理 → push 到 GitHub → WSL 端 clone 运行 |
| 验收标准 | 先跑通端到端流程，跟踪精度与工作空间分析后期迭代 |

## 未完成项

### 待外部确认（阻塞项）

1. **电机控制组的指令接口**：6 个执行器到底是位置、速度、还是力矩/电流指令。未定之前 `actuator` 层按三套并存设计。→ 需要问电机控制组。
2. **型号手册参数**：`EC-A6013`、`EC-A3814` 的角度范围与减速比；`dg4000`、`腕部2` 电缸的有效行程与额定推力。→ 查厂商手册。

### P0 · 模型

3. **拆分 B2，补回第三个肩轴**（最高优先级）。
   - `B2` = `part3 法兰3` + `part4_Joint3-Joint4_法兰1` + `法兰2_new2`（EC-A3814 输出端）
   - `B3` = `part4_肘部主支架 - 改` + `zhoubushizijia` + 肘部轴承（绕第三个肩轴转动）
   - 大臂电缸铰点从现 `R3` 顺延为 `R4`，其后所有关节名 +1
   - 影响范围：`tools_build_v2_q0.py` 的 `BODY_PARTS` / `JOINTS` / `CUTS`、`mapping.json`、`loops.yaml`、`mjcf/linear_arm.xml`、两个 README、`preview/labeled_*.png`
   - 第三肩轴的轴线与枢轴要从 CAD 配合面量（`part3 法兰3` ↔ `Joint3-Joint4_法兰` 的同心配合）
4. **关节限位**：把上表手册参数写进 URDF `<limit>` 与 MJCF `range`。旋转轴写角度范围，`L1/L2/L3` 写行程（当前完全没写，等于可任意伸缩）。
5. **加 `armature`**：质量矩阵条件数 3.1e6，某个方向惯量极小，不加会让求解器抖。
6. **加 `actuator`**：位置/速度/力矩三套并存，等第 1 项确认后选定。

### P1 · 仿真

7. **`tool0` 坐标系**：在腕心（十字轴交点）建一个显式 frame，带可配置偏置。
8. **碰撞体**：现在 `contype=0` 完全不做碰撞检测。样机验证至少要能查自身干涉，需要给 12 个刚体各做一个简化碰撞体（凸包或胶囊）。
9. **IK 与工作空间**：笛卡尔控制必需的配套分析——可达域、奇异位形、每个电缸的行程占用率。

### P2 · ROS 2

10. **接口形式未定**：是写 `ros2_control` 的 `SystemInterface` 插件（真机控制器代码零改动），还是自定义 topic。倾向前者。
11. **关节命名规范**：现在叫 `R1 R2 R3 L1 R4 R6 R7 R8 L2 R10 L3`，ROS 生态里通常用 `joint1..joint6`。改名会让所有脚本和文档一起动，趁现在定。
12. **MuJoCo 版本差异**：Windows 是 3.14.0，WSL 的 `~/venvs/robotics312` 是 3.16.0。WSL 首次跑起来要先确认 XML 两边都能加载。

## 当前状态

- 模型几何、质量、惯量已与 CAD 对齐，闭环零误差；整机 137 × 202 × 767 mm，总质量 6.3624 kg。
- **有效自由度只有 5**（见未完成项 3），设计意图是 6。
- 没有关节限位、驱动器、碰撞体和控制器。

## 历史

旧版（`deprecated/`）对不上参考图，根因有三个：SolidWorks 旋转矩阵被转置、STL 平移量用了表面形心而不是体积质心、肩部漏装第二个电机外壳。详见 `model/README.md` 第 6 节。
