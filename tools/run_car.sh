#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $EUID == 0 ]]; then echo '请用普通用户运行。' >&2; exit 2; fi
if [[ ! -f "$ROOT/ros2_ws/install/local_setup.bash" ]]; then echo '先在包内运行 bash tools/build_car.sh'; exit 2; fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/local_setup.bash"
set -u
export CROSSFIRE_NATIVE_LIBRARY="$ROOT/racelib/native/libhybrid_astar.so"
exec python3 "$ROOT/tools/car_supervisor.py" "$@"
