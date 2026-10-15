# Lua tests

The child files are ordinary pasted scripts. `build_tests.py` combines them
with the standalone test double and assertions into `combined.lua`, while
`build_module_tests.py` pastes `module_*.lua` into `ModuleExample.lua` and
generates `module_combined.lua`. From the Depths does not load child scripts
dynamically.

From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
tests/run_tests.sh
```

The generated `tests/combined.lua` and `tests/module_combined.lua` are local
test artifacts; the latter is the ModuleExample-shaped chunk to paste into
FtD after reviewing it. Use
`tests/run_tests.sh --ale` only with a local Lua runtime that provides the
`setfenv`/`getfenv` extensions expected when `_ALE` is defined.

The module integration test runs `warnOnConflict`, `silent`, `off`, warning-
plus-stop, and child-exception cases automatically. `warnOnConflict` is the
default in `ModuleExample.lua`; `silent` still records readers and writers but
does not warn or stop, while `off` skips tracking and conflict inspection.

The counterbattery-focused regression scripts are run individually with the
project-local interpreter. The shell-health contract is checked with:

```sh
.venv/bin/python tests/counterbattery_projectile_health_test.py
```

The manager keeps one endpoint tree for writers (`out`) and an equivalent tree
for readers (`readers`). Both are per-tick registries and are cleared after
conflict inspection. Child assertions inspect them before that clear.

When the real build script wraps ordinary FtD child scripts, it must insert a
fresh `local Update` declaration inside each module wrapper before pasting the
child. This preserves each script's normal top-level `function Update(I)` while
preventing one child from replacing another child's callback when `_ALE` is
unavailable.

Streaming Fourier contracts live in `fourier/streaming_fourier_test.lua` and
run through `tests/run_tests.sh` (also `--ale`). The suite rebuilds `Tides.lua`
and checks both the standalone owner and `Header.lua` + the complete paste
artifact against a direct DFT, including warmup, phase, odd sizes, long streams,
reset, reused outputs, and invalid input rejection.

Standalone target prediction contracts live in `prediction/` and run against
the source modules, generated library, and `Full/TargetPredictionComparison.lua`
in its documented paste order. Run `.venv/bin/python tests/run_target_prediction_tests.py`
with `--ale` for Lua 5.1 or `--report` for synthetic per-horizon RMSE rows.
These contracts are also part of `tests/run_tests.sh`. See
`docs/TargetPrediction.md` for models, matched-origin scoring and limitations.

`prediction/fourier_window_test.lua` verifies frequency-dependent half-lives,
weighted joint fits, phase, Nyquist, amplitude decay and invalid inputs. Prediction
contracts verify configuration propagation and window diagnostics through the launcher.
