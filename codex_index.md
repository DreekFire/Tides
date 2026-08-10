# Codex index

## Multi-script FtD acceptance tests

- `tests/build_tests.py` — concatenates the child scripts into the paste-ready `tests/combined.lua`.
- `tests/build_module_tests.py` — pastes `tests/module_*.lua` into
  `ModuleExample.lua` and generates the integration chunk.
- `tests/module_header.lua`, `tests/module_footer.lua` — FtD mock and
  assertions for the actual ModuleExample manager, including reader/writer
  registries, every override path, conflict modes, exceptions, and ALE globals.
- `tests/module_my.lua`, `tests/module_thy.lua` — ordinary child scripts with
  identical local names, isolated globals, and shared propulsion, missile,
  turret, subobject, weapon, and component endpoint access.
- `tests/run_tests.py` — rebuilds and runs the standalone chunk plus the module
  mode/exception matrix through `lupa`; `--ale` selects Lua 5.1 for fenv tests.
- `tests/combined_header.lua` — local FtD API test double and ROS-style reader/writer tracker.
- `tests/turret_aim.lua`, `tests/turret_fire.lua`, `tests/subobject_aim.lua`, `tests/subobject_fire.lua`, `tests/hull_weapon.lua` — isolated scripts exercising shared turret, ordinary subobject, and hull weapon endpoints.
- `tests/component_writer.lua`, `tests/component_reader.lua` — isolated scripts exercising a shared component endpoint.
- `tests/combined_footer.lua` — cross-script assertions and `_ALE`-gated global-context assertions.
- `ModuleExample.lua` — example module wrappers; each wrapper predeclares
  `Update`, and the real build script must insert that local before each child.

### Reusable verification workflow

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
tests/run_tests.sh
```

Use `tests/run_tests.sh --ale` when the local runtime provides the ALE
`setfenv`/`getfenv` extensions and `_ALE` should be defined.

## NdArray iterator layers

- `Low/NdArray.lua` — minimal scalar `NdArray.nditer`, direct-axis
  `NdArray.coreIter`, and single-output `NdArray.gufunc`. Operands use the
  generic `{shape, stride, data, offset}` representation; user-facing axis
  indices are zero-based and `-1` in `nditer` `opAxes` means newaxis.
- `tests/ndarray/ndarray_core_iterator_test.lua` — broadcasting, complete
  `opAxes`, reusable core views, exact gufunc core dimensions, and output-axis
  placement.
- `tests/run_ndarray_core_iterator_tests.py` — reproducible Lua/lupa test
  runner; run it with `.venv/bin/python tests/run_ndarray_core_iterator_tests.py`.

The layers are intentionally one-way: scalar iteration does not expose core
views; `coreIter` takes positional core-axis lists and broadcasts only loop
axes; `gufunc` parses a signature, validates named core sizes, and delegates
loop traversal to the same normalized representation. The current scalar
layer is read-only and returns reused index/value tables.

### NdArray verification workflow

```sh
.venv/bin/python tests/run_ndarray_core_iterator_tests.py
.venv/bin/python Util/concat.py
```
