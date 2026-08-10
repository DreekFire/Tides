@echo off
set "REPO_ROOT=%~dp0.."
"%REPO_ROOT%\.venv\Scripts\python.exe" "%REPO_ROOT%\tests\run_tests.py" %*
