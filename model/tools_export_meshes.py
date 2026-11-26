"""从 CAD 零件导出合并后的 B0..B12 STL（单位：米）。

只导出 ``model/meshes/`` 下的 13 个 STL 和 ``.in_meters`` 标记文件，
不写 URDF / MJCF。模板数值请用 ``tools_extract_params.py`` 校对，
XML 请用 ``tools_render_model.py`` 渲染。

用法
----
    python model/tools_export_meshes.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import trimesh

MODEL_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(MODEL_DIR))

import cad_model  # noqa: E402


def export_meshes(out_dir: Path = MODEL_DIR) -> None:
    links, _stations = cad_model.compute_q0_model()

    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    mesh_files: list[str] = []
    for i in range(13):
        bid = f"B{i}"
        link = links[bid]
        meshes = []
        for part in link["parts"]:
            mesh_mm = trimesh.load(part["stl"], force="mesh")
            mesh_mm.apply_translation(-part["shift"])
            transform = np.eye(4)
            transform[:3, :3] = part["R"]
            transform[:3, 3] = part["t"] - link["origin"]
            mesh_mm.apply_transform(transform)
            meshes.append(mesh_mm)

        merged_mm = trimesh.util.concatenate(meshes)
        com_rel_mm = link["com"] - link["origin"]
        bounds = merged_mm.bounds
        inside = np.all(com_rel_mm >= bounds[0] - 1.0) and np.all(
            com_rel_mm <= bounds[1] + 1.0
        )
        print(
            f"  {bid:3s}: m={link['m']:.4f} kg, parts={len(link['parts'])}, "
            f"com_in_bbox={inside}, ext_mm={np.round(bounds[1] - bounds[0], 1)}"
        )

        merged_m = merged_mm.copy()
        merged_m.apply_scale(0.001)
        merged_m.export(mesh_dir / f"{bid}.stl")
        mesh_files.append(f"{bid}.stl")

    with open(mesh_dir / ".in_meters", "w", encoding="utf-8", newline="\n") as f:
        json.dump({"scaled_by": 0.001, "files": mesh_files}, f, ensure_ascii=False, indent=1)

    print(f"写入 {len(mesh_files)} 个 STL 到 {mesh_dir}")


if __name__ == "__main__":
    export_meshes()
