# AGENTS.md

给在本仓库里工作的 AI 助手的交接说明。**开始干活前先读完这一页**，
再按需看 `README.md`（项目背景）和 `model/README.md`（模型细节）。

## 这个仓库是什么

一台混合驱动（直线电缸 + 旋转电机）六自由度机械臂的 URDF / MJCF 模型，
用于**样机控制器的离线验证**。机构含 3 个闭环：肘部四连杆、腕部双电缸并联。

- 13 个刚体 `B0..B12`
- 12 个树关节 + 3 个闭环约束（`R6`、`R10`、`R12`）
- 有效自由度 6
- 总质量 6.3624 kg（CAD 真值）
- 目录：`model/` 模型与 CAD 数据，`sim/` 仿真；`control/`、`vision/` 预留给电控与视觉代码

## 不要手改这些文件

它们是**生成物**，反手改会被下次构建覆盖：

```
model/urdf/linear_arm.urdf
model/mjcf/linear_arm.xml
model/loops.yaml
model/mapping.json
model/meshes/*.stl
```

要改模型，改 `model/tools_build_13body.py`，然后重新构建：

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"    # WSL 下用 python
cd D:\robotics\linrot-arm
& $PY -X utf8 .\model\tools_build_13body.py
```

## 提交前必须跑这三条

```powershell
& $PY -X utf8 .\model\tools_verify.py       # 期望：总判定: 通过
& $PY -X utf8 .\model\tools_check_deep.py   # 期望：问题汇总: 无
& $PY -X utf8 .\sim\check_model.py          # 期望：全部通过（有效自由度 6）
```

任何一条不通过，就不要提交。

## 建模约定（违反就会整体错位）

1. **`model/build/data/world_raw.json` 里的旋转是转置的**。
   `world[:3,:3]` 实际是 `R^T`，真实旋转必须用 `R_true = world[:3,:3].T`。
2. **网格按 CAD 世界姿态烘焙**，只平移不平移旋转。所以 `q=0` 时所有 body 帧同向、
   关节 `rpy` 全 0，关节轴必须写**世界方向**。
3. **URDF 根系在 B0 站位**，不是 CAD 全局原点。
   CAD 坐标换算 URDF 坐标要减去 `mapping.json["B0"]["origin_mm"]`。
4. **每个刚体的 link 原点 = 驱动它的那个关节的轴点**（构建脚本 `stations` 段）。
5. **STL 平移量用体积质心** `trimesh.center_mass`，不是 `trimesh.centroid`。
6. **`model/build/data/stl_map.json` 只存文件名**，不要写回绝对路径（WSL 下会炸）。

## 环境

- Windows 主用：`D:\ANACONDA\envs\mujoco\python.exe`（Python 3.11，MuJoCo 3.14.0）
- WSL：conda env `robot`（MuJoCo 3.16.0），数值与 Windows 逐项一致
- 依赖：numpy、trimesh、mujoco、matplotlib、Pillow

## 本机特有的坑

- **PowerShell 里多行命令、`foreach`、`while` 会被 shell 扫描器静默丢弃**：
  没有报错、没有输出、也没有执行。要么写单行，要么用写文件工具落成 `.py` 再跑。
- 中文经 PowerShell 管道会乱码，但**不影响文件内容**。判断结果看
  `OK` / `FAIL` / `通过` / `问题汇总` 这些 ASCII 或固定词。
- Git LFS push 可能报 `does not support the Git LFS locking API`，先执行：
  `git config lfs.https://github.com/weijianguizhou/linrot-arm.git/info/lfs.locksverify false`
- WSL 里 git 要走代理：`git config --global http.proxy http://127.0.0.1:7890`
  （代理客户端需切到**全局**模式，规则模式下 github.com 被判 DIRECT 而失败）

## 下一步该做什么

优先级见 `README.md` 第 6 节。当前 P0 是：

1. 关节限位（`<limit>` / `range`）— 需要型号手册数据
2. `armature` — 质量矩阵条件数 3.1e6，否则数值发飘
3. `actuator` — 等电机控制组确认指令类型

P1 是 `tool0` 腕心坐标系、碰撞体、IK 与工作空间分析。

## 代码风格

- 脚本写成可直接 `python xxx.py` 运行，路径一律用
  `os.path.dirname(os.path.abspath(__file__))` 相对定位，**不要写绝对路径**。
- 输出用 `OK` / `FAIL` / `INFO` 前缀，末行给一句总判定，方便被下游脚本抓。
- 中文注释和中文输出都可以，文件统一 UTF-8、换行 LF。
