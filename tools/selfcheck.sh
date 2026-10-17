#!/usr/bin/env bash
# Local checks by default. Existing native ROS or Docker are explicit choices.
# --quick skips the extra route/card scenarios for a fast edit-test loop.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
MODE=--local
QUICK=0
usage() {
  echo "用法: bash tools/selfcheck.sh [--local|--ros|--docker] [--quick]"
  echo "  --quick  只跑单元测试和一条 A 路线闭环，适合本地快速迭代"
  echo "  默认     单元测试 + A/B/色卡闭环；--ros 需先 ./crossfire install"
}
for arg in "$@"; do
  case "$arg" in
    --help|-h) usage; exit 0 ;;
    --quick) QUICK=1 ;;
    --local|--ros|--docker) MODE="$arg" ;;
    *) echo "未知选项: $arg" >&2; usage >&2; exit 2 ;;
  esac
done
mkdir -p output
if [[ "$MODE" == --docker ]]; then
  bash tools/test_in_container.sh python3 -m unittest discover -s tests -v 2>&1 | tee output/unit-tests-container.log
  bash tools/test_in_container.sh python3 tests/ros_native_contract.py 2>&1 | tee output/ros-native.log
  # Every run gets a unique directory; repeated selfchecks never overwrite a replay.
  bash tools/run_ros_sim.sh --scenario demo --route A --seed 3
  if [[ "$QUICK" == 1 ]]; then
    exit 0
  fi
  bash tools/run_ros_sim.sh --scenario demo --route B --seed 3
  bash tools/run_ros_sim.sh --scenario demo --route A --use-card --ai-backend fast_color --seed 3
  bash tools/run_ros_sim.sh --scenario demo --route B --use-card --ai-backend fast_color --seed 3
  exit 0
fi
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CROSSFIRE_NATIVE_LIBRARY="$ROOT/.runtime/selfcheck/libhybrid_astar.so"
bash tools/build_native.sh "$CROSSFIRE_NATIVE_LIBRARY"
python3 -B -m unittest discover -s tests -v 2>&1 | tee output/unit-tests-local.log
if [[ "$MODE" == --ros ]]; then
  bash crossfire ros-sim --route A --planner-engine native
  if [[ "$QUICK" == 1 ]]; then
    exit 0
  fi
  bash crossfire ros-sim --route B --planner-engine native
  bash crossfire ros-sim --route A --use-card --card-color red --planner-engine native
  bash crossfire ros-sim --fault estop --planner-engine native
else
  bash crossfire sim --route A --no-video --planner-engine native
  if [[ "$QUICK" == 1 ]]; then
    exit 0
  fi
  bash crossfire sim --route B --no-video --planner-engine native
  bash crossfire sim --route A --use-card --card-color red --no-video --planner-engine native
fi
