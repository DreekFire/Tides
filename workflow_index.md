# Workflow index

- Rebuild paste artifact: `.venv/bin/python Util/concat_counterbattery.py`
- Run canonical suite: `tests/run_tests.sh`
- Run coroutine manager integration: `.venv/bin/python tests/run_tests.py` (includes `ModuleExample.lua` tests)
- Run Counterbattery module contracts: `.venv/bin/python tests/counterbattery_target_tracking_test.py`, `.venv/bin/python tests/counterbattery_track_deduplication_test.py`, `.venv/bin/python tests/module_system_test.py`
- Run shell-health contract: `.venv/bin/python tests/counterbattery_projectile_health_test.py`
- Refresh downloaded ALE docs: `.venv/bin/python Util/download_atsuluaeditor_docs.py`
- Install project-local test dependencies: `.venv/bin/python -m pip install -r tests/requirements.txt`
- Canonical runtime launcher: `Full/Counterbattery/CounterBattery.lua` in FtD.

- Rebuild library: `.venv/bin/python Util/concat.py`.
- Fourier verification: `tests/run_tests.sh` (direct-DFT contracts against standalone source and Header + generated library); `--ale` repeats with Lua 5.1.
- Math verification: `.venv/bin/python tests/run_ndarray_math_tests.py`, `.venv/bin/python tests/run_ndarray_core_iterator_tests.py`, `.venv/bin/python tests/run_linalg_ndarray_tests.py`, `.venv/bin/python tests/run_stats_ndarray_tests.py`.
- Counterbattery contracts: `.venv/bin/python tests/counterbattery_target_aimpoint_test.py`, `.venv/bin/python tests/counterbattery_subobject_test.py`, `.venv/bin/python tests/counterbattery_orientation_test.py`, `.venv/bin/python tests/counterbattery_debug_visualization_test.py`.
- Solver/rotation checks: `.venv/bin/python tests/run_counterbattery_solver_tests.py`, `.venv/bin/python tests/run_target_rotation_ransac_tests.py`.
- Windows library builder: `Util/concat.bat` delegates to canonical `Util/concat.py` using `.venv\Scripts\python.exe`.
- Standalone comparison runtime: paste `Header.lua` → `Full/TargetPredictionComparison.lua` → generated `Tides.lua`; `Update(I)` observes mainframe 0, logs every 5 s.
- Prediction integration/numerical suite: `.venv/bin/python tests/run_target_prediction_tests.py` (`--ale` for Lua 5.1, `--report` for synthetic RMSE); also included by `tests/run_tests.sh`.
