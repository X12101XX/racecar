#!/usr/bin/env bash
# 用"小车同款 Python 环境"（Python 3.10 + 旧版 numpy/scipy）跑一遍全部测试。
#
# 小车是 ROS2 Humble / Ubuntu 22.04 -> Python 3.10 + numpy 1.21 + scipy 1.8 + pillow 9，
# 本机通常是更新的版本；这个脚本用来排除"我这儿能跑、车上跑不了"的版本问题。
#
#   bash tools/check_py310.sh              # 有 uv 且能联网时自动准备环境
#   PY310=/path/to/python3.10 bash tools/check_py310.sh   # 用已有的 3.10 解释器
set -uo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

PY="${PY310:-}"
VENV="/tmp/racecar-py310"

# 1) 先找现成的 3.10 解释器（含之前准备好的临时环境），避免每次都要联网
if [[ -z "$PY" ]]; then
  for cand in "$VENV/bin/python" /tmp/py310old/bin/python /tmp/py310/bin/python \
              "$(command -v python3.10 2>/dev/null || true)"; do
    if [[ -n "$cand" && -x "$cand" && "$($cand -c 'import sys;print(sys.version_info[:2]==(3,10))' 2>/dev/null)" == "True" ]]; then
      PY="$cand"; break
    fi
  done
fi

# 2) 没有就试着用 uv 现场准备一个（需要联网）
if [[ -z "$PY" ]] && command -v uv >/dev/null 2>&1; then
  echo "==> 用 uv 准备 Python 3.10 环境（$VENV，需要联网）"
  if uv python install 3.10 >/dev/null 2>&1 && uv venv --python 3.10 "$VENV" >/dev/null 2>&1; then
    uv pip install --python "$VENV/bin/python" "numpy==1.21.6" "scipy==1.8.1" "pillow==9.0.1" \
        >/dev/null 2>&1 || uv pip install --python "$VENV/bin/python" numpy scipy pillow >/dev/null 2>&1
    PY="$VENV/bin/python"
  fi
fi

if [[ -z "$PY" || ! -x "$PY" ]]; then
  cat <<'EOF'
没有找到 Python 3.10 解释器，跳过兼容性检查。两种做法：
  1) 允许联网：uv python install 3.10 && bash tools/check_py310.sh
  2) 已有 3.10：PY310=/usr/bin/python3.10 bash tools/check_py310.sh
（这一步只验证"小车同版本环境能跑"，不做也不影响其它功能）
EOF
  exit 0
fi

echo "==> 解释器：$($PY -c 'import sys;print(sys.version.split()[0], sys.executable)')"
$PY -c "import numpy, scipy, PIL, sys; print('   依赖：numpy', numpy.__version__,
       '| scipy', scipy.__version__, '| pillow', PIL.__version__)" 2>/dev/null || true
echo "==> 语法编译检查"
$PY -m compileall -q racelib sim tools tests \
    ros2_ws/src/racecar_mission/racecar_mission ros2_ws/src/racelib || exit 1
echo "==> 跑全部测试"
$PY -m unittest discover -s tests 2>&1 | grep -E "^(Ran|OK|FAILED|ERROR:|FAIL:)"
