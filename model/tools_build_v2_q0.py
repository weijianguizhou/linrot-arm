import json
import os
import shutil
import numpy as np
import trimesh
import mujoco

_MODEL = os.path.dirname(os.path.abspath(__file__))
HERE = os.path.join(os.path.dirname(_MODEL), "build", "data")
V2_DIR = _MODEL

# Exact STL positive-octant shifts (mm): p_stl - p_loc
EXACT_SHIFTS = {
    "00_直线模组手臂taotong1.STL": np.array([6.25, 0.0, 6.25]),
    "01_直线模组手臂taotong2.STL": np.array([6.25, 0.0, 6.25]),
    "05_直线模组手臂part1连接法兰2（新）.STL": np.array([39.5, 0.0, 84.45]),
    "06_直线模组手臂EC-A6013-H20-100.(20250928)。.STL": np.array([36.5, 45.0, 36.5]),
    "07_直线模组手臂part3 法兰3（新）.STL": np.array([38.5, 0.0, 0.0]),
    "08_直线模组手臂part3 法兰2（新）.STL": np.array([38.5, 0.0, 25.0]),
    "09_直线模组手臂EC-A3814-H14-107(20250928)。.STL": np.array([26.5, 50.0, 26.5]),
    "12_鱼眼轴承.STL": np.array([0.0, 15.0, 10.0]),
    "13_直线模组手臂zhoubushizijia.STL": np.array([7.0, 20.0, 5.0]),
    "16_直线模组手臂part4_Joint3-Joint4_法兰1.STL": np.array([27.0, 10.0, 27.0]),
    "17_直线模组手臂part4_肘部主支架 - 改.STL": np.array([30.0, 30.0, 3.5]),
    "18_直线模组手臂part4_肘部连接件.STL": np.array([20.0, 32.0, 25.25]),
    "19_直线模组手臂part56_腕部主支架 -改.STL": np.array([40.0, 62.716085, 234.0]),
    "20_直线模组手臂part56_腕部连接件.STL": np.array([40.0, 47.0, 20.0]),
    "21_直线模组手臂part4_Joint3-Joint4_法兰2_new2.STL": np.array([30.0, 30.0, 1.5]),
    "22_直线模组手臂part56_腕部十字轴.STL": np.array([20.0, 5.0, 5.0]),
    "23_直线模组手臂Rolamento 626Z.STL": np.array([9.5, 0.0, 9.5]),
    "24_直线模组手臂腕部2.STL": np.array([19.51806, 5.50606, 21.80537]),
    "25_直线模组手臂腕部.STL": np.array([7.5, 30.0, 10.0]),
    "27_直线模组手臂肘部连接1.STL": np.array([7.5, 7.5, 0.0]),
    "28_直线模组手臂改new_linear4000_link.STL": np.array([5.0, 15.0, 0.0]),
    "29_直线模组手臂dg4000.STL": np.array([84.80872, 29.398764, 0.0]),
    "32_直线模组手臂改new_linear4000_link2.STL": np.array([5.0, 20.0, 0.0]),
    "33_直线模组手臂ceshi_4000_link.STL": np.array([5.0, 0.0, 5.0]),
    "34_直线模组手臂ceshilinear400.STL": np.array([5.0, 5.0, 5.0]),
    "X01_直线模组手臂body固定法兰(新）.STL": np.array([53.54614, 9.0, 53.54614]),
    "X02_直线模组手臂part1连接法兰（新）.STL": np.array([25.0, 2.5, 25.0]),
}

JOINTS = [
    ("R1", "B0", "B1", "revolute"),
    ("R2", "B1", "B2", "revolute"),
    ("R3", "B2", "B3", "revolute"),
    ("L1", "B3", "B4", "prismatic"),
    ("R4", "B4", "B5", "revolute"),
    ("R5", "B2", "B5", "revolute"),
    ("R6", "B5", "B6", "revolute"),
    ("R7", "B5", "B7", "revolute"),
    ("R8", "B5", "B8", "revolute"),
    ("R9", "B6", "B11", "revolute"),
    ("L2", "B7", "B9", "prismatic"),
    ("R10", "B9", "B11", "revolute"),
    ("L3", "B8", "B10", "prismatic"),
    ("R11", "B10", "B11", "revolute"),
]
CUTS = ["R5", "R9", "R11"]

BODY_PARTS = {
    "B0": [
        "直线模组手臂右手-1/直线模组手臂EC-A6013-H20-100.(20250928)。-1",
        "直线模组手臂右手-1/直线模组直线模组手臂part1连接法兰（新）-1",
    ],
    "B1": [
        "直线模组手臂右手-1/直线模组手臂part1连接法兰2（新）-1",
        "直线模组手臂右手-1/直线模组手臂EC-A6013-H20-100.(20250928)。-2",
        "直线模组手臂右手-1/直线模组直线模组手臂part1连接法兰（新）-2",
        "直线模组手臂右手-1/直线模组手臂part3 法兰2（新）-1",
        "直线模组手臂右手-1/直线模组手臂part3 法兰3（新）-2",
        "直线模组手臂右手-1/直线模组手臂EC-A3814-H14-107(20250928)。-1",
    ],
    "B2": [
        "直线模组手臂part4_Joint3-Joint4_法兰1-1",
        "直线模组手臂part4_Joint3-Joint4_法兰2_new2-1",
        "直线模组手臂part4_肘部主支架 - 改-1",
        "直线模组手臂zhoubushizijia-1",
        "直线模组手臂taotong1-1",
        "直线模组手臂taotong1-3",
        "直线模组手臂Rolamento 626Z-2",
        "直线模组手臂Rolamento 626Z-3",
    ],
    "B3": [
        "直线模组手臂肘部-1/直线模组手臂dg4000-1",
        "直线模组手臂肘部-1/直线模组手臂改new_linear4000_link-1",
    ],
    "B4": [
        "直线模组手臂肘部-1/直线模组手臂肘部连接1-1",
        "鱼眼轴承-3",
        "直线模组手臂改new_linear4000_link2-1",
        "直线模组手臂taotong2-2",
        "直线模组手臂taotong2-3",
    ],
    "B5": [
        "直线模组手臂part4_肘部连接件-1",
        "直线模组手臂part56_腕部主支架 -改-1",
        "直线模组手臂Rolamento 626Z-5",
        "直线模组手臂Rolamento 626Z-6",
    ],
    "B6": [
        "直线模组手臂part56_腕部十字轴-1",
    ],
    "B7": [
        "直线模组手臂腕部2-1",
    ],
    "B8": [
        "直线模组手臂腕部2-3",
    ],
    "B9": [
        "直线模组手臂腕部-1",
    ],
    "B10": [
        "直线模组手臂腕部-3",
    ],
    "B11": [
        "直线模组手臂part56_腕部连接件-1",
    ],
}


def rot_between(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def compute_q0_model():
    w = json.load(open(os.path.join(HERE, "world_raw.json"), encoding="utf-8"))
    stl_map = json.load(open(os.path.join(HERE, "stl_map.json"), encoding="utf-8"))

    def get_true_rt(k):
        M = np.array(w[k]["world"])
        return M[:3, :3].T.copy(), M[:3, 3].copy() * 1000.0

    # 1. Forearm B5 straightening around elbow hinge R5
    p_R5 = np.array([-75.89814, 137.01497, -381.38833])
    R_b5m_cad, t_b5m_cad = get_true_rt("直线模组手臂part56_腕部主支架 -改-1")
    R_b5m_vert = np.array([[0.0, 1.0, 0.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    R_corr_b5 = R_b5m_vert @ R_b5m_cad.T
    t_b5m_vert = R_corr_b5 @ (t_b5m_cad - p_R5) + p_R5

    # 2. Upper arm actuator (B3/B4) alignment between R3 and straightened R4
    p_R3 = np.array([-88.89814, 137.01497, -133.88833])
    p_R4_cad = R_b5m_cad @ np.array([0.0, -20.5, 33.43]) + t_b5m_cad
    p_R4_vert = R_corr_b5 @ (p_R4_cad - p_R5) + p_R5

    u_act_cad = (p_R4_cad - p_R3) / np.linalg.norm(p_R4_cad - p_R3)
    u_act_vert = (p_R4_vert - p_R3) / np.linalg.norm(p_R4_vert - p_R3)
    R_corr_act = rot_between(u_act_cad, u_act_vert)
    delta_L1 = np.linalg.norm(p_R4_vert - p_R3) - np.linalg.norm(p_R4_cad - p_R3)

    # 3. Wrist cross-shaft (B6) & wrist flange (B11) & wrist actuators (B7..B10)
    p_R6 = R_b5m_vert @ np.array([0.0, 36.782, -214.0]) + t_b5m_vert
    p_R9 = R_b5m_vert @ np.array([0.0, 26.782, -214.0]) + t_b5m_vert
    p_R7 = R_b5m_vert @ np.array([-36.0, 0.0, -49.0]) + t_b5m_vert
    p_R8 = R_b5m_vert @ np.array([+36.0, 0.0, -49.0]) + t_b5m_vert

    R_b11_vert = np.array([[0.0, 1.0, 0.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    t_b11_vert = p_R9 - R_b11_vert @ np.array([0.0, 0.0, 21.0])

    p_R10 = R_b11_vert @ np.array([-36.0, -40.0, 21.0]) + t_b11_vert
    p_R11 = R_b11_vert @ np.array([+36.0, -40.0, 21.0]) + t_b11_vert

    u_w_r = (p_R10 - p_R7) / np.linalg.norm(p_R10 - p_R7)
    u_w_l = (p_R11 - p_R8) / np.linalg.norm(p_R11 - p_R8)

    def make_cyl_R(u, sign_x=1.0):
        y_ax = -u / np.linalg.norm(u)
        x_ax = np.array([0.0, float(sign_x), 0.0])
        z_ax = np.cross(x_ax, y_ax)
        return np.column_stack([x_ax, y_ax, z_ax])

    R_b7_vert = make_cyl_R(u_w_r, +1.0)
    t_b7_vert = p_R7 - R_b7_vert @ np.array([0.0, 120.0, 0.0])

    R_b8_vert = make_cyl_R(u_w_l, -1.0)
    t_b8_vert = p_R8 - R_b8_vert @ np.array([0.0, 120.0, 0.0])

    R_b9_vert = R_b7_vert.copy()
    t_b9_vert = p_R10 - R_b9_vert @ np.array([0.0, -20.0, 0.0])

    R_b10_vert = R_b7_vert.copy()
    t_b10_vert = p_R11 - R_b10_vert @ np.array([0.0, -20.0, 0.0])

    def get_q0_rt(k, bid):
        R_cad, t_cad = get_true_rt(k)
        if bid in ("B0", "B1", "B2"):
            return R_cad, t_cad
        elif bid == "B3":
            t_sym = t_cad.copy()
            t_sym[1] -= 2.0172
            return R_corr_act @ R_cad, R_corr_act @ (t_sym - p_R3) + p_R3
        elif bid == "B4":
            t_sym = t_cad.copy()
            if "直线模组手臂肘部-1" in k or "鱼眼轴承-3" in k:
                t_sym[1] -= 2.0172
            R_new = R_corr_act @ R_cad
            t_new = R_corr_act @ (t_sym - p_R3) + p_R3 + u_act_vert * delta_L1
            return R_new, t_new
        elif bid in ("B5", "B6"):
            return R_corr_b5 @ R_cad, R_corr_b5 @ (t_cad - p_R5) + p_R5
        elif bid == "B7":
            return R_b7_vert, t_b7_vert
        elif bid == "B8":
            return R_b8_vert, t_b8_vert
        elif bid == "B9":
            return R_b9_vert, t_b9_vert
        elif bid == "B10":
            return R_b10_vert, t_b10_vert
        elif bid == "B11":
            return R_b11_vert, t_b11_vert

    # Joint stations & axes in vertical q=0 pose
    p_R1 = np.array([-88.89814, 78.26497, 33.11167])
    p_R2 = np.array([-88.89814, 137.01497, -71.50000])
    p_L1 = p_R3 + u_act_vert * (218.68 + delta_L1)
    p_L2 = p_R7 + u_w_r * 120.0
    p_L3 = p_R8 + u_w_l * 120.0

    stations = {
        "R1":  dict(p=p_R1,       a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心配合轴线 (肩部第1轴)"),
        "R2":  dict(p=p_R2,       a=np.array([0.0, 0.0, 1.0]),  src="SolidWorks 同心配合轴线 (肩部第2轴)"),
        "R3":  dict(p=p_R3,       a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心529/530 (大臂电缸上支点)"),
        "L1":  dict(p=p_L1,       a=u_act_vert,                 src="SolidWorks 同心553 (大臂电缸直线移动副)"),
        "R4":  dict(p=p_R4_vert,  a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心552/554 (大臂电缸下支点)"),
        "R5":  dict(p=p_R5,       a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心527/322/323 (肘部主铰链闭环)"),
        "R6":  dict(p=p_R6,       a=np.array([-1.0, 0.0, 0.0]), src="SolidWorks 同心34/417 (腕部十字轴第1轴)"),
        "R7":  dict(p=p_R7,       a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心496/497 (腕部右电缸上支点)"),
        "R8":  dict(p=p_R8,       a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心499/500 (腕部左电缸上支点)"),
        "R9":  dict(p=p_R9,       a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心216/217 (腕部十字轴第2轴闭环)"),
        "L2":  dict(p=p_L2,       a=u_w_r,                      src="SolidWorks 同心473 (腕部右电缸直线移动副)"),
        "R10": dict(p=p_R10,      a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心477 (腕部右推杆下支点)"),
        "L3":  dict(p=p_L3,       a=u_w_l,                      src="SolidWorks 同心481 (腕部左电缸直线移动副)"),
        "R11": dict(p=p_R11,      a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心487 (腕部左推杆下支点闭环)"),
    }

    parent_joint = {ch: jn for jn, par, ch, jt in JOINTS if jn not in CUTS}

    links = {}
    for bid, keys in BODY_PARTS.items():
        part_objs = []
        for k in keys:
            v = w[k]
            mp = v.get("mass_props") or []
            R_q0, t_q0 = get_q0_rt(k, bid)
            f = stl_map.get(k)
            bname = os.path.basename(f)
            shift = EXACT_SHIFTS[bname]
            if len(mp) >= 12 and mp[5] and mp[5] > 0:
                m_val = float(mp[5])
                c_loc = np.array(mp[0:3]) * 1000.0
                com_q0 = R_q0 @ c_loc + t_q0
                I_loc = np.array([[mp[6], mp[9], mp[10]],
                                  [mp[9], mp[7], mp[11]],
                                  [mp[10], mp[11], mp[8]]])
                I_q0 = R_q0 @ I_loc @ R_q0.T
            else:
                m_val = 0.0
                com_q0 = t_q0.copy()
                I_q0 = np.zeros((3, 3))
            part_objs.append(dict(
                key=k, name=k.split("/")[-1], stl=f, shift=shift,
                R=R_q0, t=t_q0, m=m_val, com=com_q0, I=I_q0
            ))
        m_tot = sum(p["m"] for p in part_objs)
        com_tot = sum(p["m"] * p["com"] for p in part_objs) / m_tot
        I_tot = np.zeros((3, 3))
        for p in part_objs:
            if p["m"] > 0:
                d = (p["com"] - com_tot) / 1000.0
                I_tot += p["I"] + p["m"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        pj = parent_joint.get(bid)
        origin = stations[pj]["p"].copy() if pj else stations["R1"]["p"].copy()
        links[bid] = dict(m=m_tot, com=com_tot, I=I_tot, origin=origin, parts=part_objs)

    return links, stations


def build_all(out_dir=V2_DIR):
    links, stations = compute_q0_model()

    mesh_dir = os.path.join(out_dir, "meshes")
    bak_dir = os.path.join(out_dir, "meshes_mm_backup")
    urdf_dir = os.path.join(out_dir, "urdf")
    mjcf_dir = os.path.join(out_dir, "mjcf")
    os.makedirs(mesh_dir, exist_ok=True)
    os.makedirs(bak_dir, exist_ok=True)
    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(mjcf_dir, exist_ok=True)

    # 1. Export meshes in mm (backup) and m (primary)
    mesh_files = []
    for i in range(12):
        bid = f"B{i}"
        L = links[bid]
        ms = []
        for p in L["parts"]:
            mm = trimesh.load(p["stl"], force="mesh")
            mm.apply_translation(-p["shift"])
            T = np.eye(4)
            T[:3, :3] = p["R"]
            T[:3, 3] = p["t"] - L["origin"]
            mm.apply_transform(T)
            ms.append(mm)
        merged_mm = trimesh.util.concatenate(ms)
        # Check COM inside bounds
        com_rel_mm = L["com"] - L["origin"]
        b_mm = merged_mm.bounds
        inside = np.all(com_rel_mm >= b_mm[0] - 1.0) and np.all(com_rel_mm <= b_mm[1] + 1.0)
        print(f"  {bid:3s}: m={L['m']:.4f} kg, parts={len(L['parts'])}, com_in_bbox={inside}, ext_mm={np.round(b_mm[1]-b_mm[0],1)}")
        merged_mm.export(os.path.join(bak_dir, f"{bid}.stl"))

        merged_m = merged_mm.copy()
        merged_m.apply_scale(0.001)
        merged_m.export(os.path.join(mesh_dir, f"{bid}.stl"))
        L["mesh"] = f"../meshes/{bid}.stl"
        mesh_files.append(f"{bid}.stl")

    json.dump({"scaled_by": 0.001, "files": mesh_files},
              open(os.path.join(mesh_dir, ".in_meters"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    # 2. Write URDF
    def fmt(v):
        return " ".join(f"{float(x):.6g}" for x in v)

    out = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<!-- 直线模组手臂 v2（精确对齐 手臂动力学模型.png 竖直标准零位 q=0）：',
        '     * 12 刚体（B0..B11），14 关节（11 树关节 + 3 闭环关节 R5/R9/R11）',
        '     * 修正了 SolidWorks IMathTransform 旋转矩阵转置问题与 STL 体积质心偏移',
        '     * 补全肩部双 EC-A6013 电机、抱箍与大臂电缸上下连接件 -->',
        '<robot name="linear_arm">',
        '  <link name="base_link" />',
        '  <joint name="base_joint" type="fixed">',
        '    <parent link="base_link" />',
        '    <child link="B0" />',
        '    <origin xyz="0 0 0" rpy="0 0 0" />',
        '  </joint>',
    ]
    for i in range(12):
        bid = f"B{i}"
        L = links[bid]
        I = L["I"]
        com_rel_m = (L["com"] - L["origin"]) / 1000.0
        out.append(f'  <link name="{bid}">')
        out.append('    <visual>')
        out.append('      <origin xyz="0 0 0" rpy="0 0 0" />')
        out.append(f'      <geometry><mesh filename="{L["mesh"]}" /></geometry>')
        out.append('      <material name="grey"><color rgba="0.7 0.7 0.72 1" /></material>')
        out.append('    </visual>')
        out.append('    <collision>')
        out.append('      <origin xyz="0 0 0" rpy="0 0 0" />')
        out.append(f'      <geometry><mesh filename="{L["mesh"]}" /></geometry>')
        out.append('    </collision>')
        out.append('    <inertial>')
        out.append(f'      <origin xyz="{fmt(com_rel_m)}" rpy="0 0 0" />')
        out.append(f'      <mass value="{L["m"]:.6f}" />')
        out.append(f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" '
                   f'iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}" />')
        out.append('    </inertial>')
        out.append('  </link>')

    for jn, par, ch, jt in JOINTS:
        if jn in CUTS:
            continue
        s = stations[jn]
        off_m = (s["p"] - links[par]["origin"]) / 1000.0
        ax_u = s["a"] / np.linalg.norm(s["a"])
        out.append(f'  <!-- {jn}: {s["src"]} -->')
        out.append(f'  <joint name="{jn}" type="{jt}">')
        out.append(f'    <parent link="{par}" />')
        out.append(f'    <child link="{ch}" />')
        out.append(f'    <origin xyz="{fmt(off_m)}" rpy="0 0 0" />')
        out.append(f'    <axis xyz="{fmt(ax_u)}" />')
        if jt == "prismatic":
            out.append('    <limit lower="-0.1" upper="0.1" effort="200" velocity="0.5" />')
        else:
            out.append('    <limit lower="-2.9" upper="2.9" effort="60" velocity="3.0" />')
        out.append('  </joint>')
    out.append('</robot>')

    urdf_path = os.path.join(urdf_dir, "linear_arm.urdf")
    open(urdf_path, "w", encoding="utf-8").write("\n".join(out) + "\n")

    # 3. Write loops.yaml
    loops = []
    for jn, par, ch, jt in JOINTS:
        if jn in CUTS:
            s = stations[jn]
            par_o = links[par]["origin"]
            ch_o = links[ch]["origin"]
            ax_u = s["a"] / np.linalg.norm(s["a"])
            loops.append(dict(
                joint=jn,
                parent=par,
                child=ch,
                type=jt,
                origin_mm=[round(float(x), 3) for x in s["p"]],
                origin_parent_mm=[round(float(x), 3) for x in (s["p"] - par_o)],
                origin_child_mm=[round(float(x), 3) for x in (s["p"] - ch_o)],
                axis=[round(float(x), 6) for x in ax_u],
                note="闭环约束关节：URDF 树里没有，需在 MuJoCo/SDF 用 equality/joint 补"
            ))
    json.dump(loops, open(os.path.join(out_dir, "loops.yaml"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    # 4. Write mapping.json
    mp_out = {}
    for i in range(12):
        bid = f"B{i}"
        L = links[bid]
        mp_out[bid] = dict(
            mass_kg=round(float(L["m"]), 6),
            com_mm=[round(float(x), 2) for x in L["com"]],
            parts=[p["name"] for p in L["parts"]],
            origin_mm=[round(float(x), 2) for x in L["origin"]],
            mesh=f"meshes/{bid}.stl"
        )
    json.dump(mp_out, open(os.path.join(out_dir, "mapping.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    # 5. Write MJCF (linear_arm.xml)
    def body_xml(bid, indent=4):
        pad = " " * indent
        L = links[bid]
        com_rel_m = (L["com"] - L["origin"]) / 1000.0
        I = L["I"]
        # Use fullinertia for exact inertia tensor representation
        fi = [I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]]
        lines = []
        if bid == "B0":
            lines.append(f'{pad}<body name="B0" pos="0 0 0">')
        else:
            pj = [jn for jn, par, ch, jt in JOINTS if ch == bid and jn not in CUTS][0]
            par = [par for jn, par, ch, jt in JOINTS if jn == pj][0]
            jt = [jt for jn, par, ch, jt in JOINTS if jn == pj][0]
            s = stations[pj]
            off_m = (s["p"] - links[par]["origin"]) / 1000.0
            ax_u = s["a"] / np.linalg.norm(s["a"])
            jtype = "slide" if jt == "prismatic" else "hinge"
            jrange = "-0.1 0.1" if jt == "prismatic" else "-2.9 2.9"
            lines.append(f'{pad}<body name="{bid}" pos="{fmt(off_m)}">')
            lines.append(f'{pad}  <joint name="{pj}" type="{jtype}" pos="0 0 0" axis="{fmt(ax_u)}" range="{jrange}"/>')
        lines.append(f'{pad}  <inertial pos="{fmt(com_rel_m)}" mass="{L["m"]:.6f}" fullinertia="{fmt(fi)}"/>')
        lines.append(f'{pad}  <geom name="{bid}_geom" mesh="{bid}_mesh"/>')
        # Recurse children in tree
        for jn, par, ch, jt in JOINTS:
            if par == bid and jn not in CUTS:
                lines.extend(body_xml(ch, indent + 2))
        lines.append(f'{pad}</body>')
        return lines

    mjcf_lines = [
        '<mujoco model="linear_arm">',
        '  <compiler angle="radian" meshdir="../meshes" autolimits="true"/>',
        '  <option gravity="0 0 -9.81"/>',
        '  <visual>',
        '    <global offwidth="1400" offheight="1400"/>',
        '    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>',
        '    <map znear="0.01" zfar="20"/>',
        '  </visual>',
        '  <default>',
        '    <joint damping="0.05" armature="0.0"/>',
        '    <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.72 0.74 0.77 1"/>',
        '  </default>',
        '  <asset>',
    ]
    for i in range(12):
        mjcf_lines.append(f'    <mesh name="B{i}_mesh" file="B{i}.stl" scale="1 1 1"/>')
    mjcf_lines.append('  </asset>')
    mjcf_lines.append('  <worldbody>')
    mjcf_lines.extend(body_xml("B0", indent=4))
    mjcf_lines.append('  </worldbody>')
    mjcf_lines.append('')
    mjcf_lines.append('  <!-- 闭环：URDF 树中切断的 R5 / R9 / R11，anchor 为 body1 局部坐标 (m) -->')
    mjcf_lines.append('  <equality>')
    for lp in loops:
        jn, par, ch = lp["joint"], lp["parent"], lp["child"]
        anc_m = np.array(lp["origin_parent_mm"]) / 1000.0
        mjcf_lines.append(f'    <!-- {jn}: {par} -> {ch} (revolute, axis={lp["axis"]}) -->')
        mjcf_lines.append(f'    <connect name="loop_{jn}" body1="{par}" body2="{ch}" anchor="{fmt(anc_m)}"/>')
    mjcf_lines.append('  </equality>')
    mjcf_lines.append('</mujoco>')

    mjcf_path = os.path.join(mjcf_dir, "linear_arm.xml")
    open(mjcf_path, "w", encoding="utf-8").write("\n".join(mjcf_lines) + "\n")
    print("Successfully built URDF, MJCF, meshes, loops.yaml, and mapping.json in:", out_dir)

if __name__ == "__main__":
    build_all()
