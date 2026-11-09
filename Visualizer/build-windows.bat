@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel% equ 0 (
  set "PYTHON=py -3"
) else (
  set "PYTHON=python"
)
if not exist ".build-venv\Scripts\python.exe" (
  %PYTHON% -m venv .build-venv
  if errorlevel 1 goto failed
)
".build-venv\Scripts\python.exe" -m pip install -r requirements-build.txt
if errorlevel 1 goto failed
".build-venv\Scripts\python.exe" build_app.py --target windows %*
if errorlevel 1 goto failed
echo.
echo Build complete. Find your application and release archive in dist\native.
pause
exit /b 0
:failed
echo.
echo Build failed. Install Python 3.10+ with Tcl/Tk enabled, then read the error above.
pause
exit /b 1