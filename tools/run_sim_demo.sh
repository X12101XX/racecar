#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
for route in A B; do
  bash tools/run_ros_sim.sh --scenario demo --route "$route" --seed "${1:-1}" --outdir "/output/ros_demo_$route"
done
