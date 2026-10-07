# Environment index

- `.venv/` — project-local Python environment used by artifact generation and regression tests.
- Dependencies: `.venv/bin/python -m pip install -r tests/requirements.txt`
- Lua artifact compile check: `.venv/bin/python` with the installed `lupa` runtime.
- Target game runtime: LuaJIT/MoonSharp deployment remains external to this repository; generated artifact compilation is checked locally with Lupa.
