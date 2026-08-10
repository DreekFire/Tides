#!/usr/bin/env python3
"""Paste ordinary child scripts into ModuleExample.lua for local execution."""

from __future__ import annotations

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TEMPLATE = ROOT.parent / "ModuleExample.lua"
HEADER = ROOT / "module_header.lua"
FOOTER = ROOT / "module_footer.lua"
MARKER = "    -- paste script here"
MODULES = ("MyModule", "ThyModule")


def build(output: Path) -> None:
    children = sorted(
        path for path in ROOT.glob("module_*.lua")
        if path.name not in {HEADER.name, FOOTER.name, output.name}
    )
    if len(children) != len(MODULES):
        raise SystemExit(
            f"expected {len(MODULES)} module child scripts, found {len(children)}"
        )

    template = TEMPLATE.read_text(encoding="utf-8")
    if template.count(MARKER) != len(children):
        raise SystemExit("ModuleExample.lua does not have the expected paste slots")

    generated = template
    for module_name, child in zip(MODULES, children):
        marker_start = generated.find(MARKER)
        wrapper_start = generated.rfind("local function ", 0, marker_start)
        wrapper_prefix = generated[wrapper_start:marker_start]
        if "local Update" not in wrapper_prefix:
            raise SystemExit(f"{module_name} wrapper must declare local Update")
        slot_end = generated.find("\n    while true do", marker_start)
        if slot_end < 0:
            raise SystemExit(f"could not find end of {module_name} paste slot")
        child_text = child.read_text(encoding="utf-8").rstrip()
        replacement = (
            f"    -- BEGIN PASTED CHILD: {child.name}\n"
            f"{child_text}\n"
            f"    -- END PASTED CHILD: {child.name}"
        )
        generated = generated[:marker_start] + replacement + generated[slot_end:]

    if MARKER in generated:
        raise SystemExit("unexpanded ModuleExample paste slot")

    parts = [
        HEADER.read_text(encoding="utf-8").rstrip(),
        "\n\n-- BEGIN MODULE EXAMPLE",
        generated.rstrip(),
        "\n\n-- BEGIN MODULE TEST ASSERTIONS",
        FOOTER.read_text(encoding="utf-8").rstrip(),
        "",
    ]
    output.write_text("\n".join(parts), encoding="utf-8")
    print(f"built {output.relative_to(ROOT.parent)} from {len(children)} module scripts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "module_combined.lua")
    args = parser.parse_args()
    build(args.output.resolve())
