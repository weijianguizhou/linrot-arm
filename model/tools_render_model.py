"""把带宏的 XML 模板渲染为 MuJoCo MJCF 与 URDF。

模板语法
--------
模板顶部用一行一个的注释定义宏，正文用 ``${NAME}`` 引用：

    <!--@ B0_MASS = 0.640000 -->
    ...
    <mass value="${B0_MASS}"/>

宏定义行只存在于模板中，渲染时会被删除；产物顶部会插入“请勿手改”的
生成说明。渲染器只依赖 Python 标准库。

用法
----
    python model/tools_render_model.py
    python model/tools_render_model.py --check
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path
from string import Template

MODEL_DIR = Path(__file__).resolve().parent
TEMPLATE_DIR = MODEL_DIR / "templates"

TEMPLATES = {
    "mjcf": TEMPLATE_DIR / "linear_arm.mjcf.tpl",
    "urdf": TEMPLATE_DIR / "linear_arm.urdf.tpl",
}
OUTPUTS = {
    "mjcf": MODEL_DIR / "mjcf" / "linear_arm.xml",
    "urdf": MODEL_DIR / "urdf" / "linear_arm.urdf",
}

# <!--@ NAME = VALUE -->；VALUE 可以包含空格，但不能包含 -->
MACRO_RE = re.compile(r"^\s*<!--@\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*-->\s*$")
PLACEHOLDER_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")

GENERATED_NOTE = (
    "由 model/templates/{name} 渲染生成；修改模板后运行 "
    "model/tools_render_model.py。请勿手改本文件。"
)


class TemplateError(Exception):
    """模板语法或宏引用错误。"""


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")


def parse_template(path: Path) -> tuple[dict[str, str], str]:
    """读取模板，返回 (宏字典, 去掉宏定义行后的正文)。"""

    macros: dict[str, str] = {}
    body_lines: list[str] = []
    # 删除宏定义行后，紧跟着的空行也一并删除，避免产物顶部留下大段空白。
    skip_blank = False
    for lineno, line in enumerate(_read_text(path).splitlines(keepends=True), 1):
        match = MACRO_RE.match(line.rstrip("\n"))
        if not match:
            if skip_blank and not line.strip():
                continue
            skip_blank = False
            body_lines.append(line)
            continue
        name, value = match.group(1), match.group(2)
        if name in macros:
            raise TemplateError(f"{path}:{lineno}: 宏 {name} 重复定义")
        if "$" in value:
            raise TemplateError(f"{path}:{lineno}: 宏 {name} 的值不能包含 '$'")
        macros[name] = value
        skip_blank = True

    body = "".join(body_lines)
    body = body.lstrip("\n")
    return macros, body


def _check_placeholders(path: Path, body: str, macros: dict[str, str]) -> set[str]:
    used = set(PLACEHOLDER_RE.findall(body))
    missing = sorted(used - macros.keys())
    if missing:
        raise TemplateError(f"{path}: 以下宏未定义: {', '.join(missing)}")
    unused = sorted(macros.keys() - used)
    if unused:
        print(f"警告: {path.name} 中以下宏未被使用: {', '.join(unused)}", file=sys.stderr)
    return used


def render_template(path: Path) -> tuple[dict[str, str], str, set[str]]:
    macros, body = parse_template(path)
    used = _check_placeholders(path, body, macros)
    try:
        rendered = Template(body).substitute(macros)
    except KeyError as exc:
        raise TemplateError(f"{path}: 缺少宏 {exc.args[0]}") from exc
    except ValueError as exc:
        raise TemplateError(f"{path}: 占位符语法错误: {exc}") from exc
    return macros, rendered, used


def add_generated_note(kind: str, text: str) -> str:
    note = f"<!-- {GENERATED_NOTE.format(name=TEMPLATES[kind].name)} -->\n"
    if kind == "urdf" and text.startswith("<?xml"):
        first_newline = text.find("\n")
        if first_newline == -1:
            return text + "\n" + note
        return text[: first_newline + 1] + note + text[first_newline + 1 :]
    return note + text


def _write_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with open(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        Path(tmp_name).replace(path)
    except BaseException:
        Path(tmp_name).unlink(missing_ok=True)
        raise


def _normalize(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="只比较模板渲染结果与现有产物，不写文件；不一致时返回非零",
    )
    args = parser.parse_args(argv)

    try:
        rendered: dict[str, str] = {}
        macros: dict[str, dict[str, str]] = {}
        for kind, template_path in TEMPLATES.items():
            if not template_path.exists():
                raise TemplateError(f"模板不存在: {template_path}")
            kind_macros, text, _used = render_template(template_path)
            macros[kind] = kind_macros
            rendered[kind] = add_generated_note(kind, text)

        shared = sorted(macros["mjcf"].keys() & macros["urdf"].keys())
        mismatches = [
            name
            for name in shared
            if macros["mjcf"][name] != macros["urdf"][name]
        ]
        if mismatches:
            details = "\n".join(
                f"  {name}: mjcf={macros['mjcf'][name]!r}, urdf={macros['urdf'][name]!r}"
                for name in mismatches
            )
            raise TemplateError("两个模板的同名宏不一致:\n" + details)
    except TemplateError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2

    if args.check:
        stale: list[Path] = []
        for kind, output_path in OUTPUTS.items():
            if not output_path.exists():
                print(f"缺失: {output_path}", file=sys.stderr)
                stale.append(output_path)
                continue
            current = _normalize(output_path.read_text(encoding="utf-8"))
            if current != rendered[kind]:
                print(f"已过期: {output_path}", file=sys.stderr)
                stale.append(output_path)
        if stale:
            print("请运行: python model/tools_render_model.py", file=sys.stderr)
            return 1
        print(f"模型产物已是最新（{len(shared)} 个共享宏一致）。")
        return 0

    for kind, output_path in OUTPUTS.items():
        _write_atomic(output_path, rendered[kind])
        print(f"写入 {output_path}（{len(macros[kind])} 个宏）")
    print(f"完成；两个模板共有 {len(shared)} 个同名宏，值一致。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
