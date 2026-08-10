#!/usr/bin/env python3
from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute((ROOT / "Low" / "Linalg.lua").read_text(encoding="utf-8"))
    lua.execute((ROOT / "Low" / "NdArray.lua").read_text(encoding="utf-8"))
    lua.execute((ROOT / "tests" / "linalg" / "linalg_ndarray_test.lua").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
