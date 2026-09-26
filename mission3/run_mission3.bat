@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (set PY=.venv\Scripts\python.exe) else (set PY=python)
%PY% HUNTER_ARENA.py --mission 3 --scenario mission3_public --controller "%~dp0controller_mission3.py" --save-dir "%~dp0results" --hold-seconds 3
pause
