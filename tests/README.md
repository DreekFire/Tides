# Lua tests

The child files are ordinary pasted scripts. `build_tests.py` combines them
with the test double and assertions into `combined.lua`, because From the
Depths does not load scripts dynamically.

From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
tests/run_tests.sh
```

The generated `tests/combined.lua` is the chunk to paste into FtD. Use
`tests/run_tests.sh --ale` only with a local Lua runtime that provides the
`setfenv`/`getfenv` extensions expected when `_ALE` is defined.

When the real build script wraps ordinary FtD child scripts, it must insert a
fresh `local Update` declaration inside each module wrapper before pasting the
child. This preserves each script's normal top-level `function Update(I)` while
preventing one child from replacing another child's callback when `_ALE` is
unavailable.
