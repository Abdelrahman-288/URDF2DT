@echo off
cd /d "%~dp0"
if not exist "Windows Application\URDF2DT.exe" (
  echo The Windows application is missing. See START HERE.md.
  pause
  exit /b 1
)
start "URDF2DT" "%~dp0Windows Application\URDF2DT.exe" %*
