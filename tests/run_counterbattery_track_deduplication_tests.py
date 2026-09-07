#!/usr/bin/env python3
"""Run the focused trajectory-deduplication test."""

import runpy
from pathlib import Path


if __name__ == "__main__":
    test = Path(__file__).resolve().with_name("counterbattery_track_deduplication_test.py")
    runpy.run_path(str(test), run_name="__main__")
