#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
PY=python3; [ -x .venv/bin/python ] && PY=.venv/bin/python
$PY HUNTER_ARENA.py --mission 1 --scenario mission1_maze_C --controller "$(pwd)/controller_mission1.py" --save-dir "$(pwd)/results" --hold-seconds 3
