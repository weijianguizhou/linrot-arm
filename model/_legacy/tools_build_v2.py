"""v2：按用户新给的拓扑（B0..B11, 14 条边）和零件归属重新生成 URDF。

与 v1 的区别：
  * 12 个刚体（B0..B11），14 个关节，3 个闭环
  * 零件归属完全按用户给的清单
  * `肘部` 子装配（dg4000 / 肘部连接1 / new_linear4000_link）是浮动件，
    按 linear_4000 模组坐标系整体摆回链路（OBB 长轴对齐 + 中心对中）

站位规则（写进注释，便于核对）：
  * 有 CAD 坐标系的关节（R3/L1/R7/R8/L2/L3）→ 直接用坐标系原点与轴向
  * 其余 → 两个刚体质心的中点；转动副轴向默认 Y，棱柱副轴向取模组轴线
"""

from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = r"D:\robotics\_research"
OUTDIR = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2"

# ---------------------------------------------------------------- 拓扑（新图）
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

# ---------------------------------------------------------------- 零件归属（用户给定）
LINK_PARTS = {
    "B0": ["EC-A6013-H20-100.(20250928)。-1", "part1连接法兰2（新）-1"],
    "B1": ["EC-A3814-H14-107(20250928)。-1", "part3 法兰2（新）-1", "part3 法兰3（新）-2"],
    "B2": ["part4_Joint3-Joint4_法兰1-1", "part4_肘部主支架 - 改-1",
           "part4_Joint3-Joint4_法兰2_new2-1", "zhoubushizijia-1"],
    "B3": ["dg4000-1"],
    "B4": ["肘部连接1-1", "鱼眼轴承-3"],
    "B5": ["part4_肘部连接件-1", "part56_腕部主支架 -改-1"],
    "B6": ["part56_腕部十字轴-1"],
    "B7": ["腕部2-1"],
    "B8": ["腕部2-3"],
    "B9": ["腕部-1"],
    "B10": ["腕部-3"],
    "B11": ["part56_腕部连接件-1"],
}

# 浮动子装配里的零件：需要整体摆回链路
PARKED = {"dg4000-1", "肘部连接1-1", "new_linear4000_link-1"}

# 关节站位：用 CAD 坐标系的（其余按质心中点）
STATION_FRAME = {"R3": "linear_4000_up_trans", "L1": "linear_3000_down_trans",
                 "R7": "linear_400_r1_trans", "L2": "linear_400_r2_trans",
                 "R8": "linear_400_l1_trans", "L3": "linear_400_l2_trans"}


def load_frames():
    f = json.load(open(os.path.join(HERE, "frames.json"), encoding="utf-8"))
    out = {}
    for k, v in f["coordsys"].items():
        if isinstance(v, list):
            out[k] = {"origin": np.array([v[9], v[10], v[11]]) * 1000.0,
                      "R": np.array([v[0:3], v[3:6], v[6:9]])}
    return out


def load_parts():
    w = json.load(open(os.path.join(HERE, "world.json"), encoding="utf-8"))
    parts = []
    for name, v in w.items():
        mp = v.get("mass_props") or []
        M = np.array(v["world"])
        if len(mp) >= 12 and mp[5] and mp[5] > 0:
            com = (M[:3, :3] @ np.array(mp[0:3]) + M[:3, 3]) * 1000.0
            I = np.array([[mp[6], mp[9], mp[10]],
                          [mp[9], mp[7], mp[11]],
                          [mp[10], mp[11], mp[8]]])
            parts.append(dict(name=name.split("/")[-1], key=name,
                              path=v.get("path") or "", m=float(mp[5]),
                              com=com, I=M[:3, :3] @ I @ M[:3, :3].T, M=M))
        else:
            parts.append(dict(name=name.split("/")[-1], key=name,
                              path=v.get("path") or "", m=0.0,
                              com=M[:3, 3] * 1000.0, I=np.zeros((3, 3)), M=M))
    return parts


def rot_between(a, b):
    """把单位向量 a 转到 b 的最小旋转矩阵。"""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def compute_snap(parts, F):
    """把浮动子装配摆到 linear_4000 模组上：OBB 长轴 -> 模组轴线，中心对中。"""
    parked = [p for p in parts if any(k in p["name"] for k in PARKED)]
    if not parked:
        return np.eye(4)
    # 只用 dg4000（模组本体）算 OBB：三个件混在一起是个近立方体，长轴不可信
    ref = [p for p in parked if "dg4000" in p["name"]] or parked
    meshes = []
    for p in ref:
        f = stl_path(p)
        if not f:
            continue
        mm = trimesh.load(f, force="mesh")
        T = np.eye(4)
        T[:3, :3] = p["M"][:3, :3]
        T[:3, 3] = p["M"][:3, 3] * 1000.0
        mm.apply_transform(T)
        meshes.append(mm)
    if not meshes:
        return np.eye(4)
    merged = trimesh.util.concatenate(meshes)
    obb = merged.bounding_box_oriented
    ext = obb.primitive.extents
    ax = obb.primitive.transform[:3, :3]
    long_local = int(np.argmax(ext))
    axis_src = ax[:, long_local]
    center_src = obb.primitive.transform[:3, 3]

    up = F["linear_4000_up_trans"]["origin"]
    dn = F["linear_3000_down_trans"]["origin"]
    axis_dst = (dn - up) / np.linalg.norm(dn - up)
    center_dst = (up + dn) / 2.0

    R = rot_between(axis_src, axis_dst)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = center_dst - R @ center_src
    print(f"  snap: OBB extents={np.round(ext,1)}  长轴={np.round(axis_src,3)}"
          f" -> {np.round(axis_dst,3)}  位移={np.round(T[:3,3],1)} mm")
    return T


_STL = None


def stl_path(p):
    global _STL
    if _STL is None:
        mp = os.path.join(HERE, "stl_map.json")
        _STL = json.load(open(mp, encoding="utf-8")) if os.path.exists(mp) else {}
    hit = _STL.get(p["key"])
    if hit and os.path.exists(hit):
        return hit
    base = os.path.splitext(os.path.basename(p["path"]))[0][:12]
    d = os.path.join(HERE, "stl")
    for f in os.listdir(d):
        if base and base in f:
            return os.path.join(d, f)
    return None


def main():
    F = load_frames()
    parts = load_parts()
    by_name = {}
    for p in parts:
        by_name.setdefault(p["name"], []).append(p)

    print("计算浮动子装配的摆位 ...")
    T_snap = compute_snap(parts, F)

    # 归属
    bodies = {}
    for bid, keys in LINK_PARTS.items():
        ps = []
        for k in keys:
            cand = [p for p in parts if k in p["name"] and p not in ps]
            if not cand:
                print(f"  !! {bid}: 没找到零件 {k}")
                continue
            ps.append(cand[0])
        bodies[bid] = ps

    # 应用 snap 到浮动件
    for ps in bodies.values():
        for p in ps:
            if any(k in p["name"] for k in PARKED):
                # T_snap 的平移是 mm，p["M"] 的平移是 m —— 分开处理
                T_m = T_snap.copy()
                T_m[:3, 3] = T_m[:3, 3] / 1000.0
                p["M"] = T_m @ p["M"]
                p["com"] = (T_snap[:3, :3] @ p["com"] + T_snap[:3, 3])
                p["I"] = T_snap[:3, :3] @ p["I"] @ T_snap[:3, :3].T
                p["snapped"] = True

    # 刚体质量属性
    links = {}
    for bid, ps in bodies.items():
        if not ps:
            links[bid] = None
            continue
        m = sum(p["m"] for p in ps)
        com = (np.mean([p["com"] for p in ps], axis=0) if m <= 0
               else sum(p["m"] * p["com"] for p in ps) / m)
        I = np.zeros((3, 3))
        for p in ps:
            d = p["com"] - com
            I += p["I"] + p["m"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        links[bid] = dict(m=m, com=com, I=I, parts=ps)

    # 关节站位
    stations = {}
    for jn, par, ch, jt in JOINTS:
        if jn in STATION_FRAME:
            fr = F[STATION_FRAME[jn]]
            stations[jn] = dict(p=fr["origin"], a=None, src=f"CAD {STATION_FRAME[jn]}", conf="高")
        else:
            cp = links[par]["com"] if links.get(par) else np.zeros(3)
            cc = links[ch]["com"] if links.get(ch) else np.zeros(3)
            stations[jn] = dict(p=(cp + cc) / 2.0, a=None,
                                src="两刚体质心中点", conf="中")

    # 轴向
    up = F["linear_4000_up_trans"]["origin"]
    dn = F["linear_3000_down_trans"]["origin"]
    mod_axis = (dn - up) / np.linalg.norm(dn - up)
    r_axis = (F["linear_400_r2_trans"]["origin"] - F["linear_400_r1_trans"]["origin"])
    r_axis /= np.linalg.norm(r_axis)
    l_axis = (F["linear_400_l2_trans"]["origin"] - F["linear_400_l1_trans"]["origin"])
    l_axis /= np.linalg.norm(l_axis)
    AXIS = {"L1": mod_axis, "L2": r_axis, "L3": l_axis}
    for jn, par, ch, jt in JOINTS:
        if jt == "prismatic":
            stations[jn]["a"] = AXIS[jn]
        else:
            stations[jn]["a"] = np.array([0.0, 1.0, 0.0])

    # link 坐标系 = 父关节站位
    parent_joint = {ch: jn for jn, par, ch, jt in JOINTS if jn not in CUTS}
    for bid, L in links.items():
        if not L:
            continue
        pj = parent_joint.get(bid)
        L["origin"] = stations[pj]["p"] if pj else stations["R1"]["p"]

    # 网格
    os.makedirs(os.path.join(OUTDIR, "meshes"), exist_ok=True)
    for bid, L in links.items():
        if not L:
            continue
        ms = []
        for p in L["parts"]:
            f = stl_path(p)
            if not f:
                continue
            mm = trimesh.load(f, force="mesh")
            T = np.eye(4)
            T[:3, :3] = p["M"][:3, :3]
            T[:3, 3] = p["M"][:3, 3] * 1000.0
            mm.apply_transform(T)
            mm.apply_translation(-L["origin"])
            ms.append(mm)
        if ms:
            trimesh.util.concatenate(ms).export(
                os.path.join(OUTDIR, "meshes", f"{bid}.stl"))
            L["mesh"] = f"meshes/{bid}.stl"

    # ---------------- URDF
    def fmt(v):
        return " ".join(f"{x:.6g}" for x in v)

    out = ['<?xml version="1.0"?>',
           '<!-- 直线模组手臂 v2：拓扑按用户新给的 B0..B11 图（12 刚体 / 14 关节）',
           '     R = 转动副，L = 棱柱副；几何/质量/惯量来自 手臂改10.10.SLDASM',
           '     链路坐标系与世界系平行，关节 origin 只有平移',
           '     URDF 只能是树：闭环断点 = ' + ", ".join(CUTS) + '（见 loops.yaml） -->',
           '<robot name="linear_arm">']
    for i in range(12):
        bid = f"B{i}"
        L = links.get(bid)
        out.append(f'  <link name="{bid}">')
        if L and L.get("mesh"):
            out.append('    <visual>')
            out.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            out.append(f'      <geometry><mesh filename="{L["mesh"]}"/></geometry>')
            out.append('      <material name="grey"><color rgba="0.7 0.7 0.72 1"/></material>')
            out.append('    </visual>')
            out.append('    <collision>')
            out.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            out.append(f'      <geometry><mesh filename="{L["mesh"]}"/></geometry>')
            out.append('    </collision>')
        if L:
            I = L["I"] * 1e-6
            out.append('    <inertial>')
            out.append(f'      <origin xyz="{fmt((L["com"] - L["origin"]) / 1000.0)}" rpy="0 0 0"/>')
            out.append(f'      <mass value="{L["m"]:.6f}"/>')
            out.append(f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" '
                       f'iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>')
            out.append('    </inertial>')
        else:
            out.append('    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0"/>'
                       '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>')
        out.append('  </link>')

    for jn, par, ch, jt in JOINTS:
        if jn in CUTS:
            continue
        s = stations[jn]
        po = links[par]["origin"]
        off = (s["p"] - po) / 1000.0
        out.append(f'  <!-- {jn}: {s["src"]}（置信度 {s["conf"]}） -->')
        out.append(f'  <joint name="{jn}" type="{jt}">')
        out.append(f'    <parent link="{par}"/>')
        out.append(f'    <child link="{ch}"/>')
        out.append(f'    <origin xyz="{fmt(off)}" rpy="0 0 0"/>')
        out.append(f'    <axis xyz="{fmt(s["a"] / np.linalg.norm(s["a"]))}"/>')
        if jt == "prismatic":
            out.append('    <limit lower="-0.1" upper="0.1" effort="200" velocity="0.5"/>')
        else:
            out.append('    <limit lower="-2.9" upper="2.9" effort="60" velocity="3.0"/>')
        out.append('  </joint>')
    out.append('</robot>')

    os.makedirs(os.path.join(OUTDIR, "urdf"), exist_ok=True)
    urdf = os.path.join(OUTDIR, "urdf", "linear_arm.urdf")
    open(urdf, "w", encoding="utf-8").write("\n".join(out) + "\n")

    # loops + mapping
    loops = []
    for jn, par, ch, jt in JOINTS:
        if jn in CUTS:
            s = stations[jn]
            loops.append(dict(joint=jn, parent=par, child=ch, type=jt,
                              origin_mm=[round(float(x), 2) for x in s["p"]],
                              axis=[round(float(x), 4) for x in s["a"] / np.linalg.norm(s["a"])],
                              note="闭环约束关节：URDF 树里没有，需在 MuJoCo/SDF 用 equality/joint 补"))
    json.dump(loops, open(os.path.join(OUTDIR, "loops.yaml"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    mp = {bid: (None if not L else dict(
        mass_kg=round(L["m"], 5),
        com_mm=[round(float(x), 1) for x in L["com"]],
        parts=[p["name"][:46] for p in L["parts"]],
        origin_mm=[round(float(x), 1) for x in L["origin"]],
        mesh=L.get("mesh"))) for bid, L in links.items()}
    json.dump(mp, open(os.path.join(OUTDIR, "mapping.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    print("\nlinks:")
    tot = 0.0
    for i in range(12):
        bid = f"B{i}"
        L = links.get(bid)
        if L:
            tot += L["m"]
            print(f"  {bid:3s} {L['m']:7.4f} kg  {len(L['parts'])} 件  "
                  f"COM=({L['com'][0]:7.1f},{L['com'][1]:6.1f},{L['com'][2]:7.1f})")
        else:
            print(f"  {bid:3s} 空")
    print(f"合计 {tot:.3f} kg")
    print("wrote", urdf)


if __name__ == "__main__":
    main()
