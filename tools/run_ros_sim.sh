#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CALLER_DIR=$PWD
PROFILE=''
ARGS=()
HAS_CONFIG=false
HAS_SCENARIO=false
HAS_OUTDIR=false
ROUTE=A
RESULT_DIR=/output/ros_A
RESULT_ARGS=()
while (($#)); do
  case "$1" in
    --profile|--profile=*)
      [[ -z "$PROFILE" ]] || { echo '只能传一个 --profile' >&2; exit 2; }
      if [[ "$1" == --profile=* ]]; then PROFILE=${1#*=}; shift
      else
        (($# >= 2)) && [[ "$2" != --* ]] || { echo '--profile 需要保存名称或 JSON 文件' >&2; exit 2; }
        PROFILE=$2; shift 2
      fi
      [[ -n "$PROFILE" ]] || { echo '--profile 不能为空' >&2; exit 2; }
      ;;
    --config-dir|--config-dir=*) HAS_CONFIG=true; ARGS+=("$1"); shift ;;
    --scenario|--scenario=*) HAS_SCENARIO=true; ARGS+=("$1"); shift ;;
    --outdir)
      (($# >= 2)) && [[ "$2" != --* ]] || { echo '--outdir 需要输出目录' >&2; exit 2; }
      HAS_OUTDIR=true; RESULT_DIR=$2; ARGS+=("$1" "$2"); shift 2 ;;
    --outdir=*) HAS_OUTDIR=true; RESULT_DIR=${1#*=}; ARGS+=("$1"); shift ;;
    --route)
      (($# >= 2)) || { echo '--route 需要 A 或 B' >&2; exit 2; }
      ROUTE=$2; ARGS+=("$1" "$2"); shift 2 ;;
    --route=*) ROUTE=${1#*=}; ARGS+=("$1"); shift ;;
    -h|--help)
      cat <<'HELP'
用法：bash tools/run_ros_sim.sh [选项]
  --profile 名称或JSON文件   直接使用界面保存/导出的配置，自动加载其障碍布局
  --route A|B               指定路线（默认 A）
  --motion-preset conservative|standard|aggressive  临时覆盖运动预设
  --seed 数字               随机种子（默认 3）
  --outdir /output/目录      输出目录；--profile 默认使用带时间戳的新目录
  --max-time 秒             仿真最长时间（默认 180）
  --scenario demo|random|configured   高级场景选择（不与 --profile 同用）
  --config-dir 容器内目录    高级运行配置目录（不与 --profile 同用）
  --fault none|mission_kill|scan_drop|estop|competing_publisher

示例：bash tools/run_ros_sim.sh --profile measured_example --route A
      bash tools/run_ros_sim.sh --profile '/绝对路径/scene.json' --route B
只启动无网络、无硬件设备的隔离仿真。不会启动实车控制。
HELP
      exit 0 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
cd "$ROOT"
if [[ -n "$PROFILE" ]]; then
  if $HAS_CONFIG || $HAS_SCENARIO; then
    echo '--profile 自动选择 configured 场景；请不要同时传 --config-dir 或 --scenario' >&2
    exit 2
  fi
  SOURCE=$PROFILE
  [[ "$SOURCE" == /* ]] || SOURCE="$CALLER_DIR/$SOURCE"
  if [[ ! -f "$SOURCE" && "$PROFILE" =~ ^[A-Za-z0-9_-]{1,48}$ ]]; then
    SOURCE="$ROOT/profiles/$PROFILE.json"
  fi
  [[ -f "$SOURCE" ]] || { echo "找不到源配置 JSON：$PROFILE" >&2; exit 2; }
  mkdir -p .runtime
  SNAPSHOT=$(mktemp -d "$ROOT/.runtime/profile-XXXXXXXX")
  if ! python3 tools/profile_config.py "$SOURCE" --out "$SNAPSHOT"; then
    echo '配置校验未通过，未启动 Docker/ROS。' >&2
    exit 2
  fi
  ARGS=(--scenario configured --config-dir "/runtime/${SNAPSHOT##*/}" "${ARGS[@]}")
  LABEL=${PROFILE##*/}; LABEL=${LABEL%.json}; LABEL=${LABEL//[^a-zA-Z0-9_-]/_}; LABEL=${LABEL:0:48}
  RESULT_ARGS=(--profile-name "$LABEL")
  if ! $HAS_OUTDIR; then
    RESULT_DIR="/output/${LABEL:0:48}_${ROUTE}_$(date +%Y%m%d-%H%M%S-%N)"
    ARGS+=(--outdir "$RESULT_DIR")
    echo "仿真结果：$ROOT/output/${RESULT_DIR#/output/}"
  fi
  echo "源配置：$SOURCE（已校验并保存独立快照）"
fi
python3 tools/sync_workspace.py
mkdir -p .runtime output
STARTED_NS=$(python3 -c 'import time; print(time.time_ns())')
STATUS=0
docker run --rm --init --network none --read-only --cap-drop ALL --security-opt no-new-privileges --pids-limit 256 \
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
  ' crossfire "${ARGS[@]}" || STATUS=$?
python3 tools/sim_results.py "$RESULT_DIR" --since-ns "$STARTED_NS" "${RESULT_ARGS[@]}" || echo '结果入口未更新，请查看上方本次输出目录。' >&2
exit "$STATUS"
