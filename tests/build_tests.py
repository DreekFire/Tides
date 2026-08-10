#!/usr/bin/env python3
"""Build the paste-ready test chunk; FtD cannot load child scripts dynamically."""

from __future__ import annotations

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parent
HEADER = ROOT / "combined_header.lua"
FOOTER = ROOT / "combined_footer.lua"


def build(output: Path) -> None:
    children = sorted(ROOT.glob("*.lua"))
    children = [
        path for path in children
        if path.name not in {HEADER.name, FOOTER.name, output.name}
        and not path.name.startswith("module_")
    ]
    if not children:
        raise SystemExit("no child Lua tests found")

    parts = [HEADER.read_text(encoding="utf-8").rstrip()]
    for child in children:
        parts.append("\n\n-- BEGIN CHILD TEST: " + child.name)
        parts.append(child.read_text(encoding="utf-8").rstrip())
        parts.append("-- END CHILD TEST: " + child.name)
    parts.append("\n\n-- BEGIN TEST ASSERTIONS")
    parts.append(FOOTER.read_text(encoding="utf-8").rstrip())
    parts.append("")
    output.write_text("\n".join(parts), encoding="utf-8")
    print(f"built {output.relative_to(ROOT.parent)} from {len(children)} child scripts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "combined.lua")
    args = parser.parse_args()
    build(args.output.resolve())
