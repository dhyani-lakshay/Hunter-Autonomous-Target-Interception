@echo off
setlocal
cd /d "%~dp0"
echo ========================================
echo HUNTER participant setup - Windows
 echo ========================================
python --version
python -c "import sys,struct; v=sys.version_info; print('Python 3.11:', v.major==3 and v.minor==11); print('64-bit:', struct.calcsize('P')*8==64)"
if errorlevel 1 goto FAIL
if not exist ".venv\Scripts\python.exe" (
    python -m venv .venv
    if errorlevel 1 goto FAIL
)
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install --only-binary=:all: -r requirements.txt
if errorlevel 1 goto FAIL
.venv\Scripts\python.exe -c "import pybullet,numpy; print('PyBullet: OK'); print('NumPy:', numpy.__version__)"
if errorlevel 1 goto FAIL
echo.
echo Setup complete.
pause
exit /b 0
:FAIL
echo.
echo Setup failed. Please use 64-bit Python 3.11.x.
pause
exit /b 1
