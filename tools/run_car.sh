#!/usr/bin/env bash
# Native source runtime; no colcon or Docker.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 -B "$ROOT/tools/native_runtime.py" run "$@"
