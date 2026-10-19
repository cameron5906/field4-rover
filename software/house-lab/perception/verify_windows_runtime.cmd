@echo off
setlocal
cd /d "%~dp0"
if not exist "runtime_windows\python.exe" (
  echo Missing bundled runtime_windows\python.exe. Extract the complete FIELD4 archive.
  exit /b 1
)
"runtime_windows\python.exe" "probe_runtime.py" --pose --output "%TEMP%\field4_perception_runtime_probe.json"
if errorlevel 1 (
  echo Runtime test failed. Review the error above.
  exit /b 1
)
echo Runtime check passed. Report: %TEMP%\field4_perception_runtime_probe.json
exit /b 0
