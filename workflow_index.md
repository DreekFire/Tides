# Workflow index

- Rebuild paste artifact: `.venv/bin/python Util/concat_counterbattery.py`
- Run canonical suite: `tests/run_tests.sh`
- Run coroutine manager integration: `.venv/bin/python tests/run_tests.py` (includes `ModuleExample.lua` tests)
- Run shell-health contract: `.venv/bin/python tests/counterbattery_projectile_health_test.py`
- Refresh downloaded ALE docs: `.venv/bin/python Util/download_atsuluaeditor_docs.py`
- Canonical runtime launcher: `Full/Counterbattery/CounterBattery.lua` in FtD.
