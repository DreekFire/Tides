# Workflow index

- Rebuild paste artifact: `.venv/bin/python Util/concat_counterbattery.py`
- Run canonical suite: `tests/run_tests.sh`
- Run coroutine manager integration: `.venv/bin/python tests/run_tests.py` (includes `ModuleExample.lua` tests)
- Run Counterbattery module contracts: `.venv/bin/python tests/counterbattery_target_tracking_test.py`, `.venv/bin/python tests/counterbattery_track_deduplication_test.py`, `.venv/bin/python tests/module_system_test.py`
- Run shell-health contract: `.venv/bin/python tests/counterbattery_projectile_health_test.py`
- Refresh downloaded ALE docs: `.venv/bin/python Util/download_atsuluaeditor_docs.py`
- Install project-local test dependencies: `.venv/bin/python -m pip install -r tests/requirements.txt`
- Canonical runtime launcher: `Full/Counterbattery/CounterBattery.lua` in FtD.
