#!/usr/bin/env python3
"""Build and execute the combined Lua test chunk with a local Lua runtime."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parent
BUILD = ROOT / "build_tests.py"
COMBINED = ROOT / "combined.lua"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ale",
        action="store_true",
        help="define _ALE before running; requires a local Lua runtime with setfenv/getfenv",
    )
    args = parser.parse_args()

    subprocess.run([sys.executable, str(BUILD)], cwd=REPO_ROOT, check=True)
    try:
        from lupa import LuaRuntime
    except ImportError:
        print("lupa is missing; run: python -m pip install -r tests/requirements.txt", file=sys.stderr)
        return 2

    lua = LuaRuntime(unpack_returned_tuples=True)
    if args.ale:
        lua.execute("_ALE = 'local-test'")
    lua.execute(COMBINED.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
