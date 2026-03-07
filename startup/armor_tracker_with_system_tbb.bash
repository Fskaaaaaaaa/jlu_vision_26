#!/usr/bin/env bash

set -euo pipefail

tracker_bin="build/linux/x86_64/release/armor_tracker"
openvino_tbb_dir="/opt/intel/openvino_2024.6.0/runtime/3rdparty/tbb/lib"
system_tbb_dirs="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"

# Keep current LD_LIBRARY_PATH but remove OpenVINO's oneTBB directory for tracker.
filtered_paths=()
IFS=':' read -r -a raw_paths <<< "${LD_LIBRARY_PATH:-}"
for p in "${raw_paths[@]}"; do
  [[ -z "${p}" ]] && continue
  [[ "${p}" == "${openvino_tbb_dir}" ]] && continue
  filtered_paths+=("${p}")
done

if [[ ${#filtered_paths[@]} -gt 0 ]]; then
  export LD_LIBRARY_PATH="${system_tbb_dirs}:$(IFS=:; echo "${filtered_paths[*]}")"
else
  export LD_LIBRARY_PATH="${system_tbb_dirs}"
fi

exec "${tracker_bin}" "$@"
