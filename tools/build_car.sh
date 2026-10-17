#!/usr/bin/env bash
# Run on the car inside a freshly extracted private release directory.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
if [[ $EUID == 0 ]]; then echo '请用普通用户运行，不要 sudo。' >&2; exit 2; fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then echo '需要已有 ROS2 Humble；此安装器不会修改系统。' >&2; exit 2; fi
python3 - <<'PY'
import hashlib,json,pathlib,sys
r=pathlib.Path.cwd()
if sys.version_info<(3,10):raise SystemExit('需要 Python >=3.10')
if (r/'MANIFEST.json').exists():
 for name,expected in json.loads((r/'MANIFEST.json').read_text()).items():
  if name.startswith(('config/','profiles/')):continue # editable data, validated at run time
  p=r/name
  if p.is_symlink() or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
   raise SystemExit('文件校验失败: '+name)
PY
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
set +u
source /opt/ros/humble/setup.bash
set -u
python3 -c 'import rclpy,numpy,cv2; from sensor_msgs.msg import LaserScan; from geometry_msgs.msg import Twist'
command -v colcon >/dev/null || { echo '现有环境缺少 colcon；未安装或修改任何系统包。'; exit 2; }
bash tools/build_native.sh "$ROOT/racelib/native/libhybrid_astar.so"
python3 tools/sync_workspace.py
cd "$ROOT/ros2_ws"
colcon build --merge-install --event-handlers console_cohesion+
printf '%s\n' '私有工作空间已构建。默认不接管底盘：bash tools/run_car.sh A'
