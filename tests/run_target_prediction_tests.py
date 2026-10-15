#!/usr/bin/env python3
"""Rebuild and verify the standalone target prediction and comparison library."""
from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "prediction"


def run(runtime, report=False):
    subprocess.run([sys.executable, str(ROOT / "Util" / "concat.py")], cwd=ROOT, check=True)
    mock = (FIXTURES / "vector_mock.lua").read_text()
    contract = ((FIXTURES / "fourier_window_test.lua").read_text() + "\n"
                + (FIXTURES / "target_prediction_test.lua").read_text())
    header = (ROOT / "Header.lua").read_text()
    standalone = "\n".join((ROOT / path).read_text() for path in (
        "Low/NdArray.lua", "Low/Linalg.lua", "Low/StreamingFourier.lua", "Low/FourierWindow.lua",
        "Medium/TargetPrediction.lua", "Medium/TargetPredictionComparison.lua"))
    for index, source in enumerate((standalone, (ROOT / "Tides.lua").read_text())):
        lua = runtime(unpack_returned_tuples=True)
        lua.globals().PREDICTION_REPORT = report and index == 1
        comparison = (FIXTURES / "target_prediction_comparison_test.lua").read_text()
        lua.execute(mock + "\n" + header + "\n" + source + "\n" + contract + "\n" + comparison)

    # Exercise the documented FtD paste order and actual Update callback.
    lua = runtime(unpack_returned_tuples=True)
    lua.execute(mock + "\n" + header + "\n"
                + (ROOT / "Full" / "TargetPredictionComparison.lua").read_text() + "\n"
                + (ROOT / "Tides.lua").read_text() + "\n"
                + (FIXTURES / "prediction_launcher_test.lua").read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ale", action="store_true", help="use Lua 5.1")
    parser.add_argument("--report", action="store_true", help="print deterministic synthetic comparison metrics")
    args = parser.parse_args()
    if args.ale:
        from lupa.lua51 import LuaRuntime
    else:
        from lupa import LuaRuntime
    run(LuaRuntime, args.report)


if __name__ == "__main__":
    main()
