# Environment index

- `.venv/` — project-local Python environment used by artifact generation and regression tests.
- Dependencies: `.venv/bin/python -m pip install -r tests/requirements.txt`
- Lua artifact compile check: `.venv/bin/python` with the installed `lupa` runtime.
- Target game runtime: LuaJIT/MoonSharp deployment remains external to this repository; generated artifact compilation is checked locally with Lupa.
- Prediction verification uses the existing `.venv` Lupa Lua 5.5/5.1 runtimes; no additional packages. Game launcher requires the FtD Vector3 API.
