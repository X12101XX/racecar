#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$ROOT/output"
exec docker run --rm --init --network none --read-only --cap-drop ALL --security-opt no-new-privileges --pids-limit 256 \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,nosuid,size=256m --tmpfs /native:rw,exec,nosuid,size=32m,mode=1777 \
 -e HOME=/tmp -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log \
 -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=89 -e PYTHONPATH=/workspace \
 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
 --mount "type=bind,src=$ROOT,dst=/workspace,readonly" -w /workspace \
 "${CROSSFIRE_SIM_IMAGE:-crossfire-sim:humble-v2}" bash -c 'set -e; source /opt/ros/humble/setup.bash; bash tools/build_native.sh /native/libhybrid_astar.so; export CROSSFIRE_NATIVE_LIBRARY=/native/libhybrid_astar.so; exec "$@"' crossfire "$@"
