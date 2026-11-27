# AGENTS.md

给在本仓库里工作的 AI 助手的交接说明。**开始干活前先读完这一页**，
再按需看 `README.md`（项目背景）和 `model/README.md`（模型细节）。

## 这个仓库是什么

一台混合驱动（直线电缸 + 旋转电机）六自由度机械臂的 URDF / MJCF 模型，
用于**样机控制器的离线验证**。机构含 3 个闭环：肘部四连杆、腕部双电缸并联。

- 13 个物理刚体 `B0..B12`；MJCF 另有 8 个鱼眼虚拟体
- 12 个物理树关节 + 6 个鱼眼铰 + 3 个闭环约束（`R6`、`R10`、`R12`）
- 任务自由度 6（总 DOF 10 = 6 任务 + 4 鱼眼/推杆被动）
- 总质量 6.3624 kg（CAD 真值）
- 目录：`model/` 模型与 CAD 数据，`sim/` 仿真；`control/`、`vision/` 预留给电控与视觉代码

## 不要手改这些文件

它们是**生成物**，反手改会被下次渲染覆盖：

```
model/urdf/linear_arm.urdf
model/mjcf/linear_arm.xml
model/loops.yaml
model/mapping.json
model/meshes/*.stl
```

模型数值的唯一源是 `model/templates/linear_arm.mjcf.tpl` 与
`model/templates/linear_arm.urdf.tpl`：模板顶部用
`<!--@ NAME = VALUE -->` 定义宏，正文用 `${NAME}` 引用。
`model/tools_extract_params.py` 只报告 CAD 漂移，不会自动覆盖模板。

改完模板后重新渲染：

```powershell
$PY = "D:\ANACONDA\envs\mujoco\python.exe"    # WSL 下用 python
cd D:\robotics\linrot-arm
& $PY -X utf8 .\model\tools_extract_params.py --diff   # 可选：模板数值 vs CAD
& $PY -X utf8 .\model\tools_export_meshes.py           # 仅 CAD 网格变化时
& $PY -X utf8 .\model\tools_render_model.py            # 模板 -> URDF / MJCF
```

如果 CAD 的质量属性或站位确实变了，先跑
`tools_extract_params.py --write-metadata` 重建 `loops.yaml` / `mapping.json`，
再手工把新宏值同步进模板。

## 提交前必须跑这三条

```powershell
& $PY -X utf8 .\model\tools_verify.py       # 期望：总判定: 通过
& $PY -X utf8 .\model\tools_check_deep.py   # 期望：问题汇总: 无
& $PY -X utf8 .\sim\check_model.py          # 期望：全部通过（任务自由度 6）
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

1. 关节限位（`<limit>` / `range`）— R2 单向 `[0, +2.9] rad`、L1 单向 `[−0.057, 0] m` 已收紧，其余仍为占位值（±2.9 rad / ±0.1 m），等型号手册确认
2. `armature` — 质量矩阵条件数 3.1e6，否则数值发飘
3. 执行器参数 — 6 个 `<position>` 执行器已启用（占位增益/力限），等电机控制组确认指令类型后替换

P1 是 `tool0` 腕心坐标系、碰撞体、IK 与工作空间分析。

## 代码风格

- 脚本写成可直接 `python xxx.py` 运行，路径一律用
  `os.path.dirname(os.path.abspath(__file__))` 相对定位，**不要写绝对路径**。
- 输出用 `OK` / `FAIL` / `INFO` 前缀，末行给一句总判定，方便被下游脚本抓。
- 中文注释和中文输出都可以，文件统一 UTF-8、换行 LF。
