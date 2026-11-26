"""从 CAD 计算并打印/校对模板顶部的宏定义。

默认只打印宏定义块，不写任何文件：

    python model/tools_extract_params.py

校对模板（CAD 结果与模板中冻结的宏是否一致）：

    python model/tools_extract_params.py --diff

如果 CAD 确实更新了，可显式重建 ``loops.yaml`` 与 ``mapping.json``：

    python model/tools_extract_params.py --write-metadata

模板是模型数值的唯一可编辑源；本脚本不会自动覆盖模板。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cad_model  # noqa: E402


def _fmt_vector(values) -> str:
    parts = []
    for value in values:
        text = f"{float(value):.6g}"
        if text == "-0":
            text = "0"
        parts.append(text)
    return " ".join(parts)


def _fmt_mass(value: float) -> str:
    text = f"{float(value):.6f}"
    return "0.000000" if text == "-0.000000" else text


def _fmt_inertia(value: float) -> str:
    text = f"{float(value):.6e}"
    return "0.000000e+00" if text == "-0.000000e+00" else text


def generate_macros() -> dict[str, str]:
    links, stations = cad_model.compute_q0_model()

    macros: dict[str, str] = {
        # 全局
        "COLOR_BODY": "0.7 0.7 0.72 1",
        "JOINT_DAMPING": "0.05",
        "JOINT_ARMATURE": "0.0",
        "REV_LOWER": "-2.9",
        "REV_UPPER": "2.9",
        "REV_EFFORT": "60",
        "REV_VELOCITY": "3.0",
        "SLIDE_LOWER": "-0.1",
        "SLIDE_UPPER": "0.1",
        "SLIDE_EFFORT": "200",
        "SLIDE_VELOCITY": "0.5",
    }

    # 刚体质量、质心、惯量
    for i in range(13):
        bid = f"B{i}"
        link = links[bid]
        inertia = link["I"]
        macros[f"{bid}_MASS"] = _fmt_mass(link["m"])
        macros[f"{bid}_COM"] = _fmt_vector((link["com"] - link["origin"]) / 1000.0)
        macros[f"{bid}_IXX"] = _fmt_inertia(inertia[0, 0])
        macros[f"{bid}_IYY"] = _fmt_inertia(inertia[1, 1])
        macros[f"{bid}_IZZ"] = _fmt_inertia(inertia[2, 2])
        macros[f"{bid}_IXY"] = _fmt_inertia(inertia[0, 1])
        macros[f"{bid}_IXZ"] = _fmt_inertia(inertia[0, 2])
        macros[f"{bid}_IYZ"] = _fmt_inertia(inertia[1, 2])

    # 树关节原点与轴（零原点省略，例如 R1）
    for joint_name, parent, _child, _joint_type in cad_model.TREE_JOINTS:
        station = stations[joint_name]
        offset_m = (station["p"] - links[parent]["origin"]) / 1000.0
        if np.linalg.norm(offset_m) > 1e-12:
            macros[f"{joint_name}_ORIGIN"] = _fmt_vector(offset_m)
        axis = station["a"] / np.linalg.norm(station["a"])
        macros[f"{joint_name}_AXIS"] = _fmt_vector(axis)

    # 闭环约束 anchor（parent 局部坐标，m；与旧 loops.yaml 一样先四舍五入到 mm）
    for joint_name, parent, _child, _joint_type in cad_model.JOINTS:
        if joint_name not in cad_model.CUTS:
            continue
        station = stations[joint_name]
        anchor_mm = [round(float(x), 3) for x in (station["p"] - links[parent]["origin"])]
        macros[f"LOOP_{joint_name}_ANCHOR"] = _fmt_vector(np.array(anchor_mm) / 1000.0)

    return macros


def format_macro_block(macros: dict[str, str]) -> str:
    return "\n".join(f"<!--@ {name} = {value} -->" for name, value in macros.items())


def _diff_templates(macros: dict[str, str]) -> int:
    from tools_render_model import TEMPLATES, TemplateError, parse_template

    parsed: dict[str, dict[str, str]] = {}
    problems = 0
    for kind, path in TEMPLATES.items():
        if not path.exists():
            print(f"模板不存在: {path}", file=sys.stderr)
            problems += 1
            continue
        try:
            template_macros, _body = parse_template(path)
        except TemplateError as exc:
            print(f"错误: {exc}", file=sys.stderr)
            problems += 1
            continue
        parsed[kind] = template_macros
        print(f"{path.name}: {len(template_macros)} 个宏")

    if problems:
        print("模板未更新。若确认 CAD 变更，请手工同步上面的宏值。", file=sys.stderr)
        return 1

    all_template_names = set().union(*(m.keys() for m in parsed.values()))
    missing = sorted(macros.keys() - all_template_names)
    extra = sorted(all_template_names - macros.keys())
    changed: list[tuple[str, str, str]] = []
    for name in sorted(macros.keys() & all_template_names):
        for kind, template_macros in parsed.items():
            if name in template_macros and template_macros[name] != macros[name]:
                changed.append((name, kind, template_macros[name]))

    if not missing and not extra and not changed:
        print("两个模板的宏与 CAD 一致。")
        return 0

    print("模板与 CAD 不一致", file=sys.stderr)
    for name in missing:
        print(f"  两个模板都缺少: {name} = {macros[name]}", file=sys.stderr)
    for name in extra:
        print(f"  模板多出: {name}", file=sys.stderr)
    for name, kind, old_value in changed:
        print(
            f"  数值变化: {name} ({kind}): 模板 {old_value} -> CAD {macros[name]}",
            file=sys.stderr,
        )
    print("模板未更新。若确认 CAD 变更，请手工同步上面的宏值。", file=sys.stderr)
    return 1


def _write_metadata() -> None:
    links, stations = cad_model.compute_q0_model()
    model_dir = Path(cad_model.MODEL_DIR)

    loops = []
    for joint_name, parent, child, joint_type in cad_model.JOINTS:
        if joint_name not in cad_model.CUTS:
            continue
        station = stations[joint_name]
        parent_origin = links[parent]["origin"]
        child_origin = links[child]["origin"]
        axis = station["a"] / np.linalg.norm(station["a"])
        loops.append(dict(
            joint=joint_name,
            parent=parent,
            child=child,
            type=joint_type,
            origin_mm=[round(float(x), 3) for x in station["p"]],
            origin_parent_mm=[round(float(x), 3) for x in (station["p"] - parent_origin)],
            origin_child_mm=[round(float(x), 3) for x in (station["p"] - child_origin)],
            axis=[round(float(x), 6) for x in axis],
            note="闭环约束关节：URDF 树里没有，需在 MuJoCo/SDF 用 equality/joint 补",
        ))

    loops_path = model_dir / "loops.yaml"
    loops_path.write_text(
        json.dumps(loops, ensure_ascii=False, indent=1) + "\n",
        encoding="utf-8",
        newline="\n",
    )

    mapping = {}
    for i in range(13):
        bid = f"B{i}"
        link = links[bid]
        mapping[bid] = dict(
            mass_kg=round(float(link["m"]), 6),
            com_mm=[round(float(x), 2) for x in link["com"]],
            parts=[part["name"] for part in link["parts"]],
            origin_mm=[round(float(x), 2) for x in link["origin"]],
            mesh=f"meshes/{bid}.stl",
        )

    mapping_path = model_dir / "mapping.json"
    mapping_path.write_text(
        json.dumps(mapping, ensure_ascii=False, indent=1) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(f"写入 {loops_path}")
    print(f"写入 {mapping_path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--diff", action="store_true", help="比较 CAD 宏与两个模板中的冻结值")
    parser.add_argument(
        "--write-metadata",
        action="store_true",
        help="显式重建 loops.yaml 与 mapping.json（默认不写）",
    )
    args = parser.parse_args(argv)

    macros = generate_macros()
    if args.diff:
        if _diff_templates(macros) != 0:
            return 1
    else:
        print(format_macro_block(macros))

    if args.write_metadata:
        _write_metadata()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
