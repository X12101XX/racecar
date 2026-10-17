#!/usr/bin/env bash
# Compatibility entry: local release generation only. No SSH/rsync/remote execution.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -gt 0 ]]; then echo '部署脚本只做本地打包。请查看 README 的独立部署步骤；不会自动连接或驱动小车。' >&2; exit 2; fi
exec python3 "$ROOT/tools/package_release.py"
