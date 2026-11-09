"""按 手臂动力学拓扑图.png 的 B0..B12 拓扑，从 SolidWorks 提取的几何/质量生成 URDF。

数据来源：
  frames.json   —— 装配里 12 个命名坐标系（关节站位，作者自己放的）
  world.json    —— 53 个组件的世界变换 + 质量属性（质量/质心/惯量，SI）
  stl/          —— 25 个零件的网格（mm）

约定（为了让模型好核对，全部链路坐标系与世界系平行）：
  * 每个 link 的坐标系原点 = 它的父关节站位点，姿态 = 世界姿态
  * joint origin 于是只有平移，rpy = 0
  * visual / inertial 的 origin 用同一套平移
惯量张量按世界姿态写（因为 link 姿态 = 世界姿态，不用再转一次）。
"""

from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, "linear_arm_urdf")

# ---------------------------------------------------------------- 拓扑
# (joint, parent, child, type)  —— 与 手臂动力学拓扑图.png 一致
JOINTS = [
    ("R1", "B0", "B1", "revolute"),
    ("R2", "B1", "B2", "revolute"),
    ("R3", "B2", "B3", "revolute"),
    ("R4", "B3", "B4", "revolute"),
    ("L1", "B4", "B5", "prismatic"),
    ("R5", "B5", "B6", "revolute"),
    ("R6", "B3", "B6", "revolute"),
    ("R7", "B6", "B7", "revolute"),
    ("R8", "B6", "B8", "revolute"),
    ("L2", "B8", "B10", "prismatic"),
    ("R11", "B10", "B12", "revolute"),
    ("R9", "B6", "B9", "revolute"),
    ("L3", "B9", "B11", "prismatic"),
    ("R12", "B11", "B12", "revolute"),
    ("R10", "B7", "B12", "revolute"),
]
CUTS = ["R6", "R10", "R12"]          # 三个闭环各断一条，URDF 只能是树

# ---------------------------------------------------------------- 站位（mm，装配系）
# 有 CAD 坐标系的直接用；没有的按零件几何推断（source 标出来给人核对）
def load_frames():
    f = json.load(open(os.path.join(HERE, "frames.json"), encoding="utf-8"))
    out = {}
    for k, v in f["coordsys"].items():
        if not isinstance(v, list):
            continue
        o = np.array([v[9], v[10], v[11]]) * 1000.0
        R = np.array([v[0:3], v[3:6], v[6:9]])
        out[k] = {"origin": o, "R": R}
    return out


def station_table(F, parts):
    """每个关节的 (站位点, 轴向, 来源说明, 置信度)。"""
    def o(n):
        return F[n]["origin"]

    def zax(n):
        return F[n]["R"][2]

    def pc(sub):
        """按零件名片段取质心位置（没有坐标系时的替代站位）。"""
        for p in parts:
            if sub in p["name"]:
                return p["com"]
        raise KeyError(sub)

    up, mid, dn = o("linear_4000_up_trans"), o("linear_4000_trans"), o("linear_3000_down_trans")
    mod_axis = (o("linear_3000_down_trans") - o("linear_4000_up_trans"))
    mod_axis = mod_axis / np.linalg.norm(mod_axis)
    r_axis = (o("linear_400_r2_trans") - o("linear_400_r1_trans"))
    r_axis /= np.linalg.norm(r_axis)
    l_axis = (o("linear_400_l2_trans") - o("linear_400_l1_trans"))
    l_axis /= np.linalg.norm(l_axis)

    return {
        "R1": dict(p=o("base_link"),               a=np.array([0, 0, 1.0]),
                   src="base_link 坐标系的 Z（竖直回转）", conf="中"),
        "R2": dict(p=np.array([-128.3, 34.9, -49.5]), a=np.array([0, 1.0, 0]),
                   src="由 EC-A6013 电机位置推断", conf="低"),
        "R3": dict(p=np.array([-119.1, 116.9, -51.2]), a=np.array([0, 1.0, 0]),
                   src="由 EC-A6013 电机位置推断", conf="低"),
        "R4": dict(p=up, a=mod_axis,
                   src="linear_4000 上端坐标系 + 模组轴线（丝杠自转）", conf="高"),
        "L1": dict(p=dn, a=mod_axis,
                   src="linear_4000 上/下端坐标系连线（螺母行程）", conf="高"),
        "R5": dict(p=pc("part4_肘部连接件"), a=np.array([0, 1.0, 0]),
                   src="part4_肘部连接件位置推断", conf="中"),
        "R6": dict(p=mid, a=np.array([0, 1.0, 0]),
                   src="linear_4000 中段坐标系，肘部铰点", conf="中"),
        "R7": dict(p=pc("part56_腕部十字轴"), a=np.array([0, 1.0, 0]),
                   src="腕部十字轴位置推断", conf="低"),
        "R8": dict(p=o("linear_400_r1_trans"), a=r_axis,
                   src="linear_400 右模组上端 + 轴线", conf="高"),
        "L2": dict(p=o("linear_400_r2_trans"), a=r_axis,
                   src="linear_400 右模组两端连线", conf="高"),
        "R9": dict(p=o("linear_400_l1_trans"), a=l_axis,
                   src="linear_400 左模组上端 + 轴线", conf="高"),
        "L3": dict(p=o("linear_400_l2_trans"), a=l_axis,
                   src="linear_400 左模组两端连线", conf="高"),
        "R11": dict(p=o("joint5_trans"), a=zax("joint5_trans"),
                    src="joint5 坐标系", conf="高"),
        "R12": dict(p=o("joint6_trans"), a=zax("joint6_trans"),
                    src="joint6 坐标系", conf="高"),
        "R10": dict(p=pc("part56_腕部连接件"), a=np.array([0, 1.0, 0]),
                    src="joint7 只有基准轴、无坐标系；按腕部连接件位置推断", conf="低"),
    }


# ---------------------------------------------------------------- 零件 -> 刚体
# 名字用零件文件名里的关键片段匹配（world.json 的键）
LINK_PARTS = {
    "B0": ["body固定法兰", "part1连接法兰（新）"],
    "B1": ["EC-A6013-H20-100.(20250928)。-1", "part1连接法兰2（新）"],
    "B2": ["EC-A6013-H20-100.(20250928)。-2", "part3 法兰3（新）"],
    "B3": ["part4_肘部主支架", "part4_Joint3-Joint4_法兰1",
           "part4_Joint3-Joint4_法兰2", "m6-10.step", "part3 法兰2（新）"],
    "B4": ["taotong1", "taotong2", "ceshi_4000_link", "new_linear4000_link2",
           "zhoubushizijia", "Rolamento 626Z-2", "Rolamento 626Z-3",
           "iso 7380"],
    "B5": ["part4_肘部连接件", "鱼眼轴承"],
    "B6": ["part56_腕部主支架"],
    "B7": ["part56_腕部十字轴"],
    "B8": ["腕部2-1", "ceshilinear400-1"],
    "B9": ["腕部2-3", "ceshilinear400-2"],
    "B10": ["腕部-1", "Rolamento 626Z-5"],
    "B11": ["腕部-3", "Rolamento 626Z-6"],
    "B12": ["part56_腕部连接件", "m4-8.step"],
}


def load_world():
    w = json.load(open(os.path.join(HERE, "world.json"), encoding="utf-8"))
    parts = []
    for name, v in w.items():
        mp = v.get("mass_props") or []
        if len(mp) < 12:
            com0 = np.array(v["world"])[:3, 3] * 1000.0
            parts.append(dict(name=name.split("/")[-1], key=name,
                              path=v.get("path") or "", m=0.0, com=com0,
                              I=np.zeros((3, 3)), mass_missing=True,
                              M=np.array(v["world"])))
            continue
        if not mp[5] or mp[5] <= 0:
            continue
        M = np.array(v["world"])
        c_local = np.array(mp[0:3])
        com = (M[:3, :3] @ c_local + M[:3, 3]) * 1000.0     # mm
        R = M[:3, :3]
        # 惯量：把质心系张量转到世界姿态
        I = np.array([[mp[6], mp[9], mp[10]],
                      [mp[9], mp[7], mp[11]],
                      [mp[10], mp[11], mp[8]]])
        I_w = R @ I @ R.T
        parts.append(dict(name=name.split("/")[-1], key=name, path=v.get("path") or "",
                          m=float(mp[5]), com=com, I=I_w, M=M))
    return parts


def match(parts, keys):
    out, used = [], set()
    for k in keys:
        for p in parts:
            if k in p["name"] and p["key"] not in used:
                out.append(p)
                used.add(p["key"])
    return out


_STL_MAP = None


def stl_for(parts_list):
    """返回 [(stl 路径, part)] —— 零件网格是**零件自身坐标系**，要配世界变换。"""
    global _STL_MAP
    if _STL_MAP is None:
        mp = os.path.join(HERE, "stl_map.json")
        _STL_MAP = json.load(open(mp, encoding="utf-8")) if os.path.exists(mp) else {}
    files = os.listdir(os.path.join(HERE, "stl"))
    out = []
    for p in parts_list:
        hit = _STL_MAP.get(p["key"])
        if hit and os.path.exists(hit):
            out.append((hit, p))
            continue
        base = os.path.splitext(os.path.basename(p["path"]))[0][:12]
        for f in files:
            if base and base in f:
                out.append((os.path.join(HERE, "stl", f), p))
                break
    return out


def main():
    F = load_frames()
    parts = load_world()
    ST = station_table(F, parts)
    os.makedirs(os.path.join(OUTDIR, "meshes"), exist_ok=True)

    # 刚体质量属性
    links = {}
    for bid, keys in LINK_PARTS.items():
        ps = match(parts, keys)
        if not ps:
            links[bid] = None
            continue
        m = sum(p["m"] for p in ps)
        if m <= 0:
            com = np.mean([p["com"] for p in ps], axis=0)
        else:
            com = sum(p["m"] * p["com"] for p in ps) / m
        I = np.zeros((3, 3))
        for p in ps:
            d = p["com"] - com
            I += p["I"] + p["m"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        ap = ST[("R1" if bid == "B0" else next(j for j, p_, c, _ in JOINTS if c == bid))]["p"] \
            if bid != "B0" else ST["R1"]["p"]
        links[bid] = dict(m=m, com=com, I=I, origin=ap, parts=ps)

    # 合并网格
    for bid, L in links.items():
        if not L:
            continue
        meshes = []
        for f, p in stl_for(L["parts"]):
            try:
                mm = trimesh.load(f, force="mesh")
            except Exception:
                continue
            Tm = np.eye(4)
            Tm[:3, :3] = p["M"][:3, :3]
            Tm[:3, 3] = p["M"][:3, 3] * 1000.0        # 零件系 -> 装配系 (mm)
            mm.apply_transform(Tm)
            mm.apply_translation(-L["origin"])        # 装配系 -> link 系
            meshes.append(mm)
        if meshes:
            merged = trimesh.util.concatenate(meshes)
            merged.export(os.path.join(OUTDIR, "meshes", f"{bid}.stl"))
            L["mesh"] = f"meshes/{bid}.stl"

    # ---------------------------------------------------------------- 写 URDF
    def fmt(v):
        return " ".join(f"{x:.6g}" for x in v)

    def rpy_from_R(R):
        sx = -R[2, 1]
        sx = max(-1.0, min(1.0, sx))
        return (0.0, 0.0, 0.0)

    lines = ['<?xml version="1.0"?>',
             '<!-- 直线模组手臂：拓扑按 手臂动力学拓扑图.png（B0..B12, R=转动副 L=棱柱副）',
             '     几何/质量/惯量来自 SolidWorks 装配 手臂改10.10.SLDASM',
             f'     生成脚本 build_urdf.py；链路坐标系与世界系平行，关节 origin 只有平移',
             '     URDF 只能是树：闭环断点 = ' + ", ".join(CUTS) + '，见 loops.yaml -->',
             '<robot name="linear_arm">']

    for bid in [f"B{i}" for i in range(13)]:
        L = links.get(bid)
        lines.append(f'  <link name="{bid}">')
        if L and L.get("mesh"):
            lines.append(f'    <visual>')
            lines.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append(f'      <geometry><mesh filename="{L["mesh"]}"/></geometry>')
            lines.append(f'      <material name="grey"><color rgba="0.7 0.7 0.72 1"/></material>')
            lines.append(f'    </visual>')
            lines.append(f'    <collision>')
            lines.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append(f'      <geometry><mesh filename="{L["mesh"]}"/></geometry>')
            lines.append(f'    </collision>')
        if L:
            com_rel = L["com"] - L["origin"]
            I = L["I"] * 1e-6          # kg·mm^2 -> kg·m^2
            lines.append(f'    <inertial>')
            lines.append(f'      <origin xyz="{fmt(com_rel/1000.0)}" rpy="0 0 0"/>')
            lines.append(f'      <mass value="{L["m"]:.6f}"/>')
            lines.append(f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" '
                         f'iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>')
            lines.append(f'    </inertial>')
        else:
            lines.append('    <!-- 未匹配到 CAD 零件 -->')
            lines.append('    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.0"/>'
                         '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>')
        lines.append('  </link>')

    tree = [j for j in JOINTS if j[0] not in CUTS]
    for jn, par, ch, jt in tree:
        s = ST[jn]
        po = links[par]["origin"] if links.get(par) else np.zeros(3)
        off = (s["p"] - po) / 1000.0
        lines.append(f'  <!-- {jn}: {s["src"]}（置信度 {s["conf"]}） -->')
        lines.append(f'  <joint name="{jn}" type="{jt}">')
        lines.append(f'    <parent link="{par}"/>')
        lines.append(f'    <child link="{ch}"/>')
        lines.append(f'    <origin xyz="{fmt(off)}" rpy="0 0 0"/>')
        a = s["a"] / np.linalg.norm(s["a"])
        lines.append(f'    <axis xyz="{fmt(a)}"/>')
        if jt == "prismatic":
            lines.append('    <limit lower="-0.05" upper="0.05" effort="200" velocity="0.5"/>')
        else:
            lines.append('    <limit lower="-2.9" upper="2.9" effort="60" velocity="3.0"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    urdf = os.path.join(OUTDIR, "urdf", "linear_arm.urdf")
    os.makedirs(os.path.dirname(urdf), exist_ok=True)
    with open(urdf, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # ---------------------------------------------------------------- loops + mapping
    loops = []
    for jn, par, ch, jt in JOINTS:
        if jn in CUTS:
            s = ST[jn]
            loops.append(dict(joint=jn, parent=par, child=ch, type=jt,
                              origin_mm=[float(x) for x in s["p"] / 1000.0],
                              axis=[float(x) for x in s["a"] / np.linalg.norm(s["a"])],
                              note="闭环约束关节：URDF 树里没有，需在 MuJoCo/SDF 里用 equality/joint 补上"))
    with open(os.path.join(OUTDIR, "loops.yaml"), "w", encoding="utf-8") as f:
        json.dump(loops, f, ensure_ascii=False, indent=1)

    mp = {}
    for bid, L in links.items():
        if not L:
            mp[bid] = None
            continue
        mp[bid] = dict(mass_kg=round(L["m"], 5),
                       com_mm=[round(float(x), 2) for x in L["com"]],
                       parts=[p["name"][:48] for p in L["parts"]],
                       origin_mm=[round(float(x), 2) for x in L["origin"]],
                       mesh=L.get("mesh"))
    with open(os.path.join(OUTDIR, "mapping.json"), "w", encoding="utf-8") as f:
        json.dump(mp, f, ensure_ascii=False, indent=1)

    total = sum(L["m"] for L in links.values() if L)
    print("wrote", urdf)
    print(f"links={len(links)} total_mass={total:.3f} kg")
    for bid in [f"B{i}" for i in range(13)]:
        L = links.get(bid)
        if L:
            print(f"  {bid}: {L['m']:7.4f} kg  {len(L['parts'])} parts  mesh={L.get('mesh')}")
        else:
            print(f"  {bid}: EMPTY")


if __name__ == "__main__":
    main()
