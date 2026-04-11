#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

cd "${repo_root}"

if ! command -v xmake >/dev/null 2>&1; then
  echo "xmake 未安装，无法构建 auto_buff 离线目标。" >&2
  exit 1
fi

xmake f -y -m release
xmake build buff_detector_offline
exec process-compose --no-server -f configs/startup/auto_buff_offline_debug.yaml
