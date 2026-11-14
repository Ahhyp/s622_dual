#!/usr/bin/env bash
# Host entry: authenticate the current desktop, replay, retain the final GUI.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
gui_auth_source="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -r "$gui_auth_source" ]]; then
  printf 'Cannot read current desktop XAUTHORITY: %s\n' "$gui_auth_source" >&2
  exit 1
fi
gui_run_id="gui_$(date -u +%Y%m%dT%H%M%S)_$$"
gui_auth_target="/tmp/c43-${gui_run_id}.Xauthority"
docker cp "$gui_auth_source" "s622_yep:$gui_auth_target"
# Replays share ROS_DOMAIN_ID 146: close only our previous GUI lab processes.
docker exec s622_yep python3 -c 'import os,signal,subprocess,time,shlex
from pathlib import Path
targets=[]
for row in subprocess.check_output(["ps","-eo","pid=,args="],text=True).splitlines():
    pid,args=row.strip().split(None,1)
    parts=shlex.split(args)
    if len(parts)>=4 and Path(parts[1]).name=="c43_force_lab.py" and parts[2].startswith("/workspace/log/c43_dual_transport_20261111/gui_") and parts[3]=="dual_transport":
        targets.append(int(pid))
for pid in targets:
    try: os.kill(pid,signal.SIGINT)
    except ProcessLookupError: pass
deadline=time.monotonic()+20
while any(os.path.exists("/proc/"+str(pid)) for pid in targets) and time.monotonic()<deadline: time.sleep(.2)
if any(os.path.exists("/proc/"+str(pid)) for pid in targets): raise SystemExit("Previous GUI lab still exiting; retry shortly.")'
docker_tty=(-i)
if [[ -t 0 && -t 1 ]]; then docker_tty=(-it); fi
printf 'Logs: %s/log/c43_dual_transport_20261111/%s\n' "$repo_dir" "$gui_run_id"
printf 'Motion starts after controller setup + 20 seconds. GUI remains open after completion. Ctrl+C exits.\n'
exec docker exec "${docker_tty[@]}" -w /workspace \
  -e DISPLAY="${DISPLAY:-:4}" -e XAUTHORITY="$gui_auth_target" \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e LIBGL_ALWAYS_INDIRECT=0 \
  -e __GLX_VENDOR_LIBRARY_NAME=mesa \
  -e C43_GUI_RUN_OUT="/workspace/log/c43_dual_transport_20261111/$gui_run_id" \
  s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run gz_launch c43_run_contact_transport.py --gui --keep-open --start-delay 20 --out "$C43_GUI_RUN_OUT"'
