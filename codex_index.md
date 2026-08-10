# Codex index

## Multi-script FtD acceptance tests

- `tests/build_tests.py` — concatenates the child scripts into the paste-ready `tests/combined.lua`.
- `tests/build_module_tests.py` — pastes `tests/module_*.lua` into
  `ModuleExample.lua` and generates the integration chunk.
- `tests/module_header.lua`, `tests/module_footer.lua` — FtD mock and
  assertions for the actual ModuleExample manager.
- `tests/module_my.lua`, `tests/module_thy.lua` — ordinary child scripts with
  isolated top-level locals and shared weapon/component endpoint access.
- `tests/run_tests.py` — rebuilds and runs the combined chunk through `lupa`.
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
