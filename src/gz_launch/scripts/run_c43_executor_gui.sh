#!/usr/bin/env bash
# Host desktop entry for the physical-contact C++ executor profile.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
gui_auth_source="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -r "$gui_auth_source" ]]; then
  printf 'Cannot read desktop XAUTHORITY: %s\n' "$gui_auth_source" >&2
  exit 1
fi
gui_run_id="gui_$(date -u +%Y%m%dT%H%M%S)_$$"
gui_auth_target="/tmp/c43-executor-${gui_run_id}.Xauthority"
docker cp "$gui_auth_source" "s622_yep:$gui_auth_target"
printf 'Logs: %s/log/c43_executor_20261113/%s\n' "$repo_dir" "$gui_run_id"
printf 'Physical-contact C++ executor; GUI stays open after the task. Ctrl+C exits.\n'
exec docker exec -i -w /workspace \
  -e DISPLAY="${DISPLAY:-:4}" -e XAUTHORITY="$gui_auth_target" \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e LIBGL_ALWAYS_INDIRECT=0 \
  -e __GLX_VENDOR_LIBRARY_NAME=mesa \
  -e C43_GUI_RUN_OUT="/workspace/log/c43_executor_20261113/$gui_run_id" \
  s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run gz_launch c43_run_contact_transport.py --backend qp-executor --gui --keep-open --start-delay 5 --out "$C43_GUI_RUN_OUT"'
