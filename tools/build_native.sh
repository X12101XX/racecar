#!/usr/bin/env bash
# Build for the current host/container only. Never install into system paths.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TARGET=${1:-$ROOT/racelib/native/libhybrid_astar.so}
CXX=${CXX:-g++}
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "未找到 $CXX；auto 模式使用 Python，native 模式将报错。" >&2
  exit 0
fi
mkdir -p "$(dirname -- "$TARGET")"
TEMP=$(mktemp "${TARGET}.XXXXXXXX")
trap 'rm -f -- "$TEMP"' EXIT
"$CXX" -O3 -fPIC -shared -std=c++17 "$ROOT/racelib/native/hybrid_astar_core.cpp" -o "$TEMP"
mv -f -- "$TEMP" "$TARGET"
echo "[OK] 当前环境 C++ 规划库：$TARGET" >&2
if [[ $# == 0 ]]; then python3 "$ROOT/tools/sync_workspace.py"; fi
