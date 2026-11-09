# -*- coding: utf-8 -*-
"""生成 v2 的预览图：零位四视图 + 与参考照片的对照。

直接读 urdf/linear_arm.urdf（走 MuJoCo 的 URDF 导入），所以图里是什么，
别人拿到这份 URDF 看到的就是什么。
"""
import os

import mujoco
import numpy as np
import PIL.Image as Image
from PIL import ImageDraw, ImageFont

V2 = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(V2, "urdf", "linear_arm.urdf")
PHOTO = r"D:\robotics\手臂动力学模型.png"
OUT = os.path.join(V2, "preview")
os.makedirs(OUT, exist_ok=True)
S = 600

# 中文字体：找不到就退回默认字体（标题里只用 ASCII 也无所谓）
_FONT = None
for _p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
    if os.path.exists(_p):
        try:
            _FONT = ImageFont.truetype(_p, 20)
            break
        except OSError:
            pass


def font(size=20, bold=False):
    for p in (r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
              r"C:\Windows\Fonts\simhei.ttf"):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                pass
    return ImageFont.load_default()



def scene_points(m, d):
    pts = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = m.mesh_vert[va:va + vn]
        pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    return np.concatenate(pts)


def link_world_positions(urdf_path):
    """q=0 时各 link 的 (坐标系世界位置, 质心世界位置)。URDF 里 rpy 全 0，按平移累加即可。"""
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()])
        joints[j.find("child").get("link")] = (j.find("parent").get("link"), xyz)
    pos = {}

    def walk(link):
        if link in pos:
            return pos[link]
        if link not in joints:
            pos[link] = np.zeros(3)
        else:
            par, xyz = joints[link]
            pos[link] = walk(par) + xyz
        return pos[link]

    com = {}
    for l in root.findall("link"):
        name = l.get("name")
        walk(name)
        o = l.find("inertial/origin")
        com[name] = (pos[name] + np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()])
                     if l.find("inertial") is not None else pos[name])
    return pos, com


def project(p, cam_pos, fwd, up, fovy_deg, size):
    right = np.cross(up, fwd)
    v = np.asarray(p, float) - cam_pos
    z = float(v @ fwd)
    if z <= 1e-6:
        return None
    k = (size / 2) / np.tan(np.deg2rad(fovy_deg) / 2)
    return (size / 2 + (v @ right) / z * k, size / 2 - (v @ up) / z * k)



def main():
    m = mujoco.MjModel.from_xml_path(URDF)
    m.vis.global_.offwidth = S
    m.vis.global_.offheight = S
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    P = scene_points(m, d)
    center = (P.min(0) + P.max(0)) / 2
    radius = float(np.linalg.norm(P - center, axis=1).max())
    dist = radius / np.tan(np.deg2rad(m.vis.global_.fovy) / 2) * 1.12

    r = mujoco.Renderer(m, S, S)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = center
    cam.distance = dist

    views = [(0, -6), (90, -6), (180, -6), (270, -6)]
    imgs = []
    for az, el in views:
        cam.azimuth = az
        cam.elevation = el
        r.update_scene(d, camera=cam)
        imgs.append((az, el, Image.fromarray(r.render())))
    r.close()

    pad = 8
    canvas = Image.new("RGB", (2 * S + 3 * pad, 2 * S + 3 * pad + 24), (250, 250, 250))
    dr = ImageDraw.Draw(canvas)
    dr.text((pad, 6), "linear_arm v2  q=0  four views (metre-scaled, fixed mesh paths)",
            fill=(0, 0, 0), font=font(18))
    for i, (az, el, im) in enumerate(imgs):
        x = pad + (i % 2) * (S + pad)
        y = 24 + pad + (i // 2) * (S + pad)
        canvas.paste(im, (x, y))
        dr.text((x + 4, y + 4), f"az={az} el={el}", fill=(255, 255, 0), font=font(18))
    p1 = os.path.join(OUT, "p_zero_4view.png")
    canvas.save(p1)
    print("saved", p1)

    # 与照片对照
    photo = Image.open(PHOTO).convert("RGB")
    ph = S * 2 + 2 * pad
    photo = photo.resize((int(photo.width * ph / photo.height), ph))
    W = photo.width + 2 * S + 3 * pad
    H = ph + 3 * pad + 24
    comp = Image.new("RGB", (W, H), (250, 250, 250))
    dr = ImageDraw.Draw(comp)
    comp.paste(photo, (pad, 24 + pad))
    dr.text((pad + 2, 6), "reference: 手臂动力学模型.png", fill=(0, 0, 0), font=font(18))
    for i, (az, el, im) in enumerate(imgs[:2]):
        comp.paste(im.resize((S, S)), (photo.width + 2 * pad + i * (S + pad), 24 + pad))
        dr.text((photo.width + 2 * pad + i * (S + pad) + 4, 24 + pad + 4),
                f"az={az}", fill=(255, 255, 0), font=font(18))
    p2 = os.path.join(OUT, "compare_photo.png")
    comp.save(p2)
    print("saved", p2)
    for az, el, im in imgs:
        im.save(os.path.join(OUT, f"p_zero_az{az:03d}.png"))

    # 刚体编号标注图（B0..B11）
    cam.azimuth, cam.elevation = 0, -6
    scn = mujoco.MjvScene(m, maxgeom=20000)
    opt = mujoco.MjvOption()
    mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    cpos = np.array(scn.camera[0].pos, float)
    fwd = np.array(scn.camera[0].forward, float)
    up = np.array(scn.camera[0].up, float)
    pos, com = link_world_positions(URDF)
    lab = imgs[0][2].copy()
    dr = ImageDraw.Draw(lab)
    f12 = font(16, bold=True)
    # 左右交替 + 纵向错开，避免引线交叉
    for i in range(12):
        name = f"B{i}"
        if name not in com:
            continue
        xy = project(com[name], cpos, fwd, up, m.vis.global_.fovy, S)
        if xy is None:
            continue
        x, y = xy
        side = 1 if i % 2 == 0 else -1
        tx = x + side * (70 + 12 * (i % 3))
        ty = y + (i % 4 - 1.5) * 10
        dr.line([(x, y), (tx, ty)], fill=(200, 40, 40), width=1)
        dr.ellipse([x - 2.5, y - 2.5, x + 2.5, y + 2.5], fill=(200, 40, 40))
        tw = dr.textlength(name, font=f12)
        dr.text((tx + 3 if side > 0 else tx - tw - 3, ty - 9), name,
                fill=(200, 40, 40), font=f12)
    p3 = os.path.join(OUT, "labeled_B0_B11.png")
    lab.save(p3)
    print("saved", p3)



if __name__ == "__main__":
    main()
