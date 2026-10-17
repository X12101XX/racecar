#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
mkdir -p output
bash tools/test_in_container.sh python3 -m unittest discover -s tests -v 2>&1 | tee output/unit-tests-container.log
bash tools/test_in_container.sh python3 tests/ros_native_contract.py 2>&1 | tee output/ros-native.log
# Every run gets a unique directory; repeated selfchecks never overwrite a replay.
bash tools/run_ros_sim.sh --scenario demo --route A --seed 3
bash tools/run_ros_sim.sh --scenario demo --route B --seed 3
bash tools/run_ros_sim.sh --scenario demo --route A --use-card --ai-backend fast_color --seed 3
bash tools/run_ros_sim.sh --scenario demo --route B --use-card --ai-backend fast_color --seed 3
