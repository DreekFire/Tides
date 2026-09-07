#!/usr/bin/env python3
"""Run the focused TargetRotation RANSAC tests with the local Lua runtime."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    target_rotation = (ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetRotation.lua").read_text(
        encoding="utf-8"
    )
    tests = (ROOT / "tests" / "rotation" / "target_rotation_ransac_test.lua").read_text(encoding="utf-8")
    module = lua.execute(target_rotation + "\nreturn TargetRotation")
    lua.globals().TargetRotationModule = module
    lua.execute(tests)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
