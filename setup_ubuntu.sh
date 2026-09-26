#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 --version
python3 -c 'import sys,struct; v=sys.version_info; print("Python 3.11:", v.major==3 and v.minor==11); print("64-bit:", struct.calcsize("P")*8==64); assert v.major==3 and v.minor==11 and struct.calcsize("P")*8==64'
if [ ! -x .venv/bin/python ]; then python3 -m venv .venv; fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install --only-binary=:all: -r requirements.txt
.venv/bin/python -c 'import pybullet,numpy; print("PyBullet: OK"); print("NumPy:", numpy.__version__)'
echo 'Setup complete.'
