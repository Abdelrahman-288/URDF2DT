@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" (
  echo Install the project virtual environment first. See README.md.
  pause
  exit /b 1
)
start "URDF2DT" ".venv\Scripts\pythonw.exe" -m urdf2dt.ui.desktop %*
