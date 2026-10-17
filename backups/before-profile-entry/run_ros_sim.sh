#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
python3 tools/sync_workspace.py
mkdir -p .runtime output
exec docker run --rm --init --network none --read-only --cap-drop ALL --security-opt no-new-privileges --pids-limit 256 \
  --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,nosuid,size=512m --tmpfs /home/runner:rw,nosuid,size=32m \
  -e HOME=/home/runner -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log -e PYTHONPATH=/workspace \
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
  --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
  --mount "type=bind,src=$ROOT/.runtime,dst=/runtime" \
  --mount "type=bind,src=$ROOT/output,dst=/output" \
  -w /runtime crossfire-sim:humble-v2 bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --merge-install --base-paths /workspace/ros2_ws/src --event-handlers console_direct+
    source /runtime/install/setup.bash
    cd /workspace
    exec python3 -u -m sim.ros_demo "$@"
  ' crossfire "$@"
