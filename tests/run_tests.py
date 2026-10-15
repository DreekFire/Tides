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
MODULE_BUILD = ROOT / "build_module_tests.py"
MODULE_COMBINED = ROOT / "module_combined.lua"


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
        if args.ale:
            from lupa.lua51 import LuaRuntime
        else:
            from lupa import LuaRuntime
    except ImportError:
        print("lupa is missing; run: python -m pip install -r tests/requirements.txt", file=sys.stderr)
        return 2

    def run_chunk(combined: Path) -> None:
        lua = LuaRuntime(unpack_returned_tuples=True)
        if args.ale:
            lua.execute("_ALE = 'local-test'")
        lua.execute(combined.read_text(encoding="utf-8"))

    # Verify both the standalone owner and the actual paste-ready entry point.
    subprocess.run([sys.executable, str(REPO_ROOT / "Util" / "concat.py")],
                   cwd=REPO_ROOT, check=True)
    contract = (ROOT / "fourier" / "streaming_fourier_test.lua").read_text(encoding="utf-8")
    for source in (REPO_ROOT / "Low" / "StreamingFourier.lua", REPO_ROOT / "Tides.lua"):
        lua = LuaRuntime(unpack_returned_tuples=True)
        header = (REPO_ROOT / "Header.lua").read_text(encoding="utf-8")
        lua.execute(header + "\n" + source.read_text(encoding="utf-8") + "\n" + contract)

    run_chunk(COMBINED)

    scenarios = [
        ("warnOnConflict", False, False),
        ("silent", True, False),
        ("off", True, False),
        ("warnOnConflict", True, False),
        ("warnOnConflict", False, True),
    ]
    try:
        for mode, stop_on_conflict, module_error in scenarios:
            command = [
                sys.executable,
                str(MODULE_BUILD),
                "--mode",
                mode,
            ]
            if stop_on_conflict:
                command.append("--stop-on-conflict")
            if module_error:
                command.append("--module-error")
            subprocess.run(command, cwd=REPO_ROOT, check=True)
            run_chunk(MODULE_COMBINED)
    finally:
        # Keep the checked-in artifact in its ordinary paste-ready mode.
        subprocess.run([sys.executable, str(MODULE_BUILD)], cwd=REPO_ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
