"""CAD 几何与质量属性的共享计算。

本模块只负责从 SolidWorks 导出的 JSON 还原 q=0 竖直零位下的 13 个刚体
（B0..B12）与 15 个关节站位。它不写任何模型文件；网格导出见
``tools_export_meshes.py``，XML 渲染见 ``tools_render_model.py``。
"""

import json
import os

import numpy as np

MODEL_DIR = os.path.dirname(os.path.abspath(__file__))
HERE = os.path.join(MODEL_DIR, "build", "data")

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
    ("R1", "B0", "B1", "revolute"),      # 肩 1：EC-A6013-1
    ("R2", "B1", "B2", "revolute"),      # 肩 2：EC-A6013-2
    ("R3", "B2", "B3", "revolute"),      # 肩 3：EC-A3814（竖直）
    ("R4", "B3", "B4", "revolute"),      # 大臂电缸上支点
    ("L1", "B4", "B5", "prismatic"),     # 大臂电缸
    ("R5", "B5", "B6", "revolute"),      # 大臂电缸下支点
    ("R6", "B3", "B6", "revolute"),      # 肘部主铰链（闭环）
    ("R7", "B6", "B7", "revolute"),      # 腕十字轴 第 1 轴
    ("R8", "B6", "B8", "revolute"),      # 腕右电缸上支点
    ("R9", "B6", "B9", "revolute"),      # 腕左电缸上支点
    ("R10", "B7", "B12", "revolute"),    # 腕十字轴 第 2 轴（闭环）
    ("L2", "B8", "B10", "prismatic"),    # 腕右电缸
    ("R11", "B10", "B12", "revolute"),   # 腕右推杆下支点（闭环）
    ("L3", "B9", "B11", "prismatic"),    # 腕左电缸
    ("R12", "B11", "B12", "revolute"),   # 腕左推杆下支点（闭环）
]
CUTS = ["R6", "R10", "R12"]
TREE_JOINTS = [j for j in JOINTS if j[0] not in CUTS]

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
    ],
    "B2": [
        "直线模组手臂右手-1/直线模组手臂EC-A3814-H14-107(20250928)。-1",
        "直线模组手臂右手-1/直线模组手臂part3 法兰3（新）-2",
        "直线模组手臂part4_Joint3-Joint4_法兰1-1",
    ],
    "B3": [
        "直线模组手臂part4_Joint3-Joint4_法兰2_new2-1",
        "直线模组手臂part4_肘部主支架 - 改-1",
        "直线模组手臂zhoubushizijia-1",
        "直线模组手臂taotong1-1",
        "直线模组手臂taotong1-3",
        "直线模组手臂Rolamento 626Z-2",
        "直线模组手臂Rolamento 626Z-3",
    ],
    "B4": [
        "直线模组手臂肘部-1/直线模组手臂dg4000-1",
        "直线模组手臂肘部-1/直线模组手臂改new_linear4000_link-1",
    ],
    "B5": [
        "直线模组手臂肘部-1/直线模组手臂肘部连接1-1",
        "鱼眼轴承-3",
        "直线模组手臂改new_linear4000_link2-1",
        "直线模组手臂taotong2-2",
        "直线模组手臂taotong2-3",
    ],
    "B6": [
        "直线模组手臂part4_肘部连接件-1",
        "直线模组手臂part56_腕部主支架 -改-1",
        "直线模组手臂Rolamento 626Z-5",
        "直线模组手臂Rolamento 626Z-6",
    ],
    "B7": [
        "直线模组手臂part56_腕部十字轴-1",
    ],
    "B8": [
        "直线模组手臂腕部2-1",
    ],
    "B9": [
        "直线模组手臂腕部2-3",
    ],
    "B10": [
        "直线模组手臂腕部-1",
    ],
    "B11": [
        "直线模组手臂腕部-3",
    ],
    "B12": [
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
    with open(os.path.join(HERE, "world_raw.json"), encoding="utf-8") as f:
        w = json.load(f)
    with open(os.path.join(HERE, "stl_map.json"), encoding="utf-8") as f:
        stl_map = json.load(f)

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
        if bid in ("B0", "B1", "B2", "B3"):
            return R_cad, t_cad
        elif bid == "B4":
            t_sym = t_cad.copy()
            t_sym[1] -= 2.0172
            return R_corr_act @ R_cad, R_corr_act @ (t_sym - p_R3) + p_R3
        elif bid == "B5":
            t_sym = t_cad.copy()
            if "直线模组手臂肘部-1" in k or "鱼眼轴承-3" in k:
                t_sym[1] -= 2.0172
            R_new = R_corr_act @ R_cad
            t_new = R_corr_act @ (t_sym - p_R3) + p_R3 + u_act_vert * delta_L1
            return R_new, t_new
        elif bid in ("B6", "B7"):
            return R_corr_b5 @ R_cad, R_corr_b5 @ (t_cad - p_R5) + p_R5
        elif bid == "B8":
            return R_b7_vert, t_b7_vert
        elif bid == "B9":
            return R_b8_vert, t_b8_vert
        elif bid == "B10":
            return R_b9_vert, t_b9_vert
        elif bid == "B11":
            return R_b10_vert, t_b10_vert
        elif bid == "B12":
            return R_b11_vert, t_b11_vert
        raise ValueError(f"未知刚体: {bid}")

    # Joint stations & axes in vertical q=0 pose
    # 肩部三条轴线完全由 CAD 配合链定出：
    #   轴1 过 EC-A6013-1 原点，方向 = 该电机局部 +Y
    #   轴2 过 EC-A6013-2 原点，方向 = 该电机局部 +Y
    #   轴3 竖直，过 同心532 的 x=-88.898 / y=137.015
    U1 = np.array([0.130526, 0.991445, 0.0])
    U2 = np.array([0.991445, -0.130526, 0.0])
    U3 = np.array([0.0, 0.0, 1.0])

    p_J1 = np.array([-96.166, 39.636, 33.112])
    p_J2 = np.array([-96.329, 137.993, 33.112])
    p_J3 = np.array([-88.898, 137.015, -62.790])
    p_J4 = p_R3.copy()                      # 大臂电缸上支点
    p_L1 = p_R3 + u_act_vert * (218.68 + delta_L1)
    p_J5 = p_R4_vert.copy()                 # 大臂电缸下支点
    p_J6 = p_R5.copy()                      # 肘部主铰链（闭环）
    p_J7 = p_R6.copy()                      # 腕十字轴 第1轴
    p_J8 = p_R7.copy()                      # 腕右电缸上支点
    p_J9 = p_R8.copy()                      # 腕左电缸上支点
    p_J10 = p_R9.copy()                     # 腕十字轴 第2轴（闭环）
    p_L2 = p_R7 + u_w_r * 120.0
    p_J11 = p_R10.copy()                    # 腕右推杆下支点（闭环）
    p_L3 = p_R8 + u_w_l * 120.0
    p_J12 = p_R11.copy()                    # 腕左推杆下支点（闭环）

    stations = {
        "R1":  dict(p=p_J1,  a=U1,                     src="EC-A6013-1 轴线（肩1）"),
        "R2":  dict(p=p_J2,  a=U2,                     src="EC-A6013-2 轴线（肩2）"),
        "R3":  dict(p=p_J3,  a=U3,                     src="EC-A3814 轴线（肩3）"),
        "R4":  dict(p=p_J4,  a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心529/530/531 (大臂电缸上支点)"),
        "L1":  dict(p=p_L1,  a=u_act_vert,                 src="SolidWorks 同心553 (大臂电缸直线移动副)"),
        "R5":  dict(p=p_J5,  a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心552/554 (大臂电缸下支点)"),
        "R6":  dict(p=p_J6,  a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心527/322/323 (肘部主铰链闭环)"),
        "R7":  dict(p=p_J7,  a=np.array([-1.0, 0.0, 0.0]), src="SolidWorks 同心34/417 (腕部十字轴第1轴)"),
        "R8":  dict(p=p_J8,  a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心496/497 (腕部右电缸上支点)"),
        "R9":  dict(p=p_J9,  a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心499/500 (腕部左电缸上支点)"),
        "R10": dict(p=p_J10, a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心216/217 (腕部十字轴第2轴闭环)"),
        "L2":  dict(p=p_L2,  a=u_w_r,                      src="SolidWorks 同心473 (腕部右电缸直线移动副)"),
        "R11": dict(p=p_J11, a=np.array([0.0, 1.0, 0.0]),  src="SolidWorks 同心477 (腕部右推杆下支点闭环)"),
        "L3":  dict(p=p_L3,  a=u_w_l,                      src="SolidWorks 同心481 (腕部左电缸直线移动副)"),
        "R12": dict(p=p_J12, a=np.array([0.0, -1.0, 0.0]), src="SolidWorks 同心487 (腕部左推杆下支点闭环)"),
    }

    parent_joint = {ch: jn for jn, par, ch, jt in JOINTS if jn not in CUTS}

    links = {}
    for bid, keys in BODY_PARTS.items():
        part_objs = []
        for k in keys:
            v = w[k]
            mp = v.get("mass_props") or []
            R_q0, t_q0 = get_q0_rt(k, bid)
            f = os.path.join(HERE, "stl", os.path.basename(stl_map.get(k, "")))
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
