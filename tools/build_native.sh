#!/usr/bin/env bash
# 编译 C++ 原生 Hybrid A* 路径规划加速库
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "警告：未找到 C++ 编译器 ($CXX)，跳过原生库编译（将使用纯 Python 实现）。" >&2
    exit 0
fi

echo "正在使用 $CXX 编译 C++ 原生 Hybrid A* 路径规划加速库..."
mkdir -p racelib/native

# -O3 最高级别优化，-fPIC 生成位置无关代码，-shared 生成动态链接库
"$CXX" -O3 -fPIC -shared -std=c++17 racelib/native/hybrid_astar_core.cpp -o racelib/native/libhybrid_astar.so

echo "[OK] 编译完成：$ROOT/racelib/native/libhybrid_astar.so"
python3 tools/sync_workspace.py
