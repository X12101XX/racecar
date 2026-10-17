#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
exec docker build -t crossfire-sim:humble-v2 -f containers/Dockerfile .
