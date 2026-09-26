#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
PY=python3; [ -x .venv/bin/python ] && PY=.venv/bin/python
$PY HUNTER_ARENA.py --mission 2 --scenario mission2_public --controller "$(pwd)/controller_mission2.py" --save-dir "$(pwd)/results" --hold-seconds 3
