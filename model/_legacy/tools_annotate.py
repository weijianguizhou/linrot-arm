# -*- coding: utf-8 -*-
"""给 v2 模型出一张「同风格标注图」：白底渲染 + B0..B11 箭头标注。

原图 手臂动力学模型.png 是旧版（13 刚体 B0..B12 + 旧腕部布置），
这张按现在的 12 刚体拓扑出。
"""
import os

import mujoco
import numpy as np
import PIL.Image as Image
from PIL import ImageDraw, ImageFont

V2 = r"D:\robotics\linear_arm_model\urdf\linear_arm_v2"
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")
OUT = os.path.join(V2, "preview")
os.makedirs(OUT, exist_ok=True)
W, H = 900, 1500

TEXT = {}   # 和原图一样只标符号，不加说明



def link_centroids(m, d):
    """每个刚体的网格顶点质心（世界系）"""
    out = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        body = m.body(m.geom_bodyid[g]).name
        if body == "world":      # B0 被 fusestatic 并进 world
            body = "B0"
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = m.mesh_vert[va:va + vn]
        P = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        out.setdefault(body, []).append(P.mean(0))
    return {k: np.mean(v, axis=0) for k, v in out.items()}


def main():
    m = mujoco.MjModel.from_xml_path(URDF)
    m.vis.global_.offwidth = W
    m.vis.global_.offheight = H
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    pts = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = m.mesh_vert[va:va + vn]
        pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    P = np.concatenate(pts)
    lo, hi = P.min(0), P.max(0)
    center = (lo + hi) / 2
    radius = float(np.linalg.norm(P - center, axis=1).max())
    fovy = np.deg2rad(m.vis.global_.fovy)
    aspect = W / H
    dist = radius / np.tan(fovy / 2) * 1.05

    AZ, EL = 150.0, -6.0
    cam = mujoco.MjvCamera()
    cam.lookat[:] = center
    cam.distance = dist
    cam.azimuth = AZ
    cam.elevation = EL

    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, camera=cam)
    img = r.render()
    r.close()

    # 直接从 MuJoCo 场景拿相机位姿，避免自己猜约定
    opt = mujoco.MjvOption()
    scene = mujoco.MjvScene(m, maxgeom=400)
    mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    c0 = scene.camera[0]
    cpos = np.array(c0.pos)
    fwd = np.array(c0.forward) / np.linalg.norm(c0.forward)
    upv = np.array(c0.up) / np.linalg.norm(c0.up)
    right = np.cross(fwd, upv)
    right /= np.linalg.norm(right)

    def project(X):
        v = X - cpos
        z = float(v @ fwd)
        x = float(v @ right)
        y = float(v @ upv)
        u = 0.5 + x / (2 * z * np.tan(fovy / 2) * aspect)
        w = 0.5 - y / (2 * z * np.tan(fovy / 2))
        return u * W, w * H, z

    # 白底，放进更宽的画布：左边留标注栏，模型靠右
    mask = img.max(axis=2) > 8
    model = np.ones_like(img) * 255
    model[mask] = img[mask]
    MW = 1320
    canvas = Image.new("RGB", (MW, H), (255, 255, 255))
    canvas.paste(Image.fromarray(model).convert("RGB"), (420, 0))
    dr = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("msyh.ttc", 40)
        font_s = ImageFont.truetype("msyh.ttc", 20)
    except Exception:
        font = font_s = ImageFont.load_default()

    cents = link_centroids(m, d)
    items = []
    for name, X in cents.items():
        u, v, z = project(X)
        items.append([v, u + 420, name, 0.0])
    items.sort()
    gap = min(96, (H - 110) // max(1, len(items) - 1))
    last = -1e9
    for it in items:
        ty = max(it[0] - 20, last + gap)
        last = ty
        it[3] = ty
    over = items[-1][3] - (H - 90) if items else 0
    if over > 0:                     # 整体上移，避免最后一栏被裁掉
        for it in items:
            it[3] = max(16, it[3] - over)
    for v, u, name, ty in items:
        txt = TEXT.get(name, "")
        dr.line([(340, ty + 22), (u, v)], fill=(120, 120, 120), width=2)
        dr.ellipse([u - 5, v - 5, u + 5, v + 5], fill=(200, 30, 30))
        dr.text((24, ty), name, fill=(0, 0, 0), font=font)
        if txt:
            dr.text((24, ty + 33), txt, fill=(80, 80, 80), font=font_s)

    p = os.path.join(OUT, "annotated_B0_B11.png")
    canvas.save(p)
    root = os.path.join(OUT, "annotated_root.png")
    canvas.crop((420, 0, 420 + W, H)).save(root)
    print("saved", p)
    for name, X in sorted(cents.items()):
        u, v, z = project(X)
        print(f"  {name:4s} -> ({u:5.0f},{v:5.0f})  z={z:.3f}")


if __name__ == "__main__":
    main()
