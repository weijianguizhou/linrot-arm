"""校验 URDF（XML/树/质量）+ 把各 link 摆到零位渲染出来看形状。"""

import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "linear_arm_urdf")
URDF = os.path.join(ROOT, "urdf", "linear_arm.urdf")


def parse():
    tree = ET.parse(URDF)
    robot = tree.getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {}
    for j in robot.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        joints[j.get("name")] = dict(
            type=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()]),
            axis=np.array([float(x) for x in (a.get("xyz") if a is not None else "1 0 0").split()]),
        )
    return robot, links, joints


def check(links, joints):
    errs = []
    for n, j in joints.items():
        if j["parent"] not in links:
            errs.append(f"{n}: parent {j['parent']} missing")
        if j["child"] not in links:
            errs.append(f"{n}: child {j['child']} missing")
    kids = {}
    for n, j in joints.items():
        if j["child"] in kids:
            errs.append(f"{j['child']} 有两个父关节")
        kids[j["child"]] = n
    roots = [l for l in links if l not in kids]
    if len(roots) != 1:
        errs.append(f"根节点数 = {len(roots)}: {roots}")
    # 连通性
    seen, stack = set(), roots[:1]
    adj = {}
    for n, j in joints.items():
        adj.setdefault(j["parent"], []).append(j["child"])
    while stack:
        x = stack.pop()
        if x in seen:
            errs.append(f"环 at {x}")
            break
        seen.add(x)
        stack.extend(adj.get(x, []))
    if len(seen) != len(links):
        errs.append(f"不可达 link: {sorted(set(links) - seen)}")
    # 质量
    total = 0.0
    for n, l in links.items():
        m = l.find("inertial/mass")
        if m is not None:
            total += float(m.get("value"))
    return errs, total, roots


def fk(links, joints):
    poses = {}
    kids = {j["child"]: n for n, j in joints.items()}
    root = [l for l in links if l not in kids][0]
    poses[root] = np.eye(4)

    def rec(link):
        for n, j in joints.items():
            if j["parent"] == link:
                T = np.eye(4)
                T[:3, 3] = j["xyz"] * 1000.0
                poses[j["child"]] = poses[link] @ T
                rec(j["child"])

    rec(root)
    return poses


def main():
    robot, links, joints = parse()
    errs, total, roots = check(links, joints)
    print("links:", len(links), "joints:", len(joints), "root:", roots,
          "total mass: %.3f kg" % total)
    print("types:", {t: sum(1 for j in joints.values() if j["type"] == t)
                     for t in ("revolute", "prismatic", "fixed")})
    print("errors:", errs if errs else "none")

    poses = fk(links, joints)
    fig, axes = plt.subplots(1, 2, figsize=(20, 13))
    for ax, (i, j2, title) in zip(axes, [(0, 2, "X-Z"), (1, 2, "Y-Z")]):
        for name, l in links.items():
            vis = l.find("visual/geometry/mesh")
            if vis is None or name not in poses:
                continue
            f = os.path.join(ROOT, vis.get("filename"))
            if not os.path.exists(f):
                continue
            mesh = trimesh.load(f, force="mesh")
            T = poses[name]
            mesh.apply_translation(T[:3, 3])
            pts = mesh.sample(min(1200, max(150, len(mesh.faces) // 3)))
            ax.scatter(pts[:, i], pts[:, j2], s=1.2, alpha=0.55)
            c = mesh.centroid
            ax.annotate(name, (c[i], c[j2]), fontsize=9,
                        xytext=(3, 3), textcoords="offset points")
        ax.set_title(f"{title}  (mm)")
        ax.set_xlabel("XYZ"[i])
        ax.set_ylabel("XYZ"[j2])
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "urdf_check.png"), dpi=120)
    print("saved", os.path.join(ROOT, "urdf_check.png"))
    return 0 if not errs else 1


if __name__ == "__main__":
    sys.exit(main())
