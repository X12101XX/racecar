#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
mkdir -p output
bash tools/test_in_container.sh python3 -m unittest discover -s tests -v 2>&1 | tee output/unit-tests-container.log
bash tools/test_in_container.sh python3 tests/ros_native_contract.py 2>&1 | tee output/ros-native.log
bash tools/run_ros_sim.sh --scenario demo --route A --seed 1 --outdir /output/ros_demo_A
bash tools/run_ros_sim.sh --scenario demo --route B --seed 1 --outdir /output/ros_demo_B
