#!/usr/bin/env bash
# This environment exists only in our child process. Never source ~/.bashrc.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SETUP=$1
PYTHON=$2
shift 2
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME
unset VIRTUAL_ENV CONDA_PREFIX BASH_ENV ENV
set +u
source "$SETUP"
set -u
export PYTHONPATH="$ROOT:$ROOT/ros2_ws/src/racecar_mission${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CROSSFIRE_NATIVE_LIBRARY="$ROOT/.runtime/car/libhybrid_astar.so"
export CROSSFIRE_NODE_ENTRY="$ROOT/tools/node_entry.py"
exec "$PYTHON" "$@"
