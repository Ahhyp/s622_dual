#!/usr/bin/env bash
# C4.1 one-shot reproduction: sample the dual-arm configuration space and
# regenerate every figure, table and summary number quoted in
# C4.1_执行记录_工作空间与可操作度.md
#
#   bash src/dual_arm_workspace_analysis/analysis/run_c41.sh
#
# Environment overrides:
#   SAMPLES=10000000   configuration pairs to evaluate
#   D_MIN=0.02         arm-arm rejection threshold [m]
#   THREADS=0          0 = all cores
#   SENS=1             also run the rank-tol / d_min sensitivity sweeps
#   SKIP_BUILD=1       do not run colcon build first
#   PY=...             python interpreter with numpy+matplotlib
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG_NAME="dual_arm_workspace_analysis"

SAMPLES="${SAMPLES:-10000000}"
D_MIN="${D_MIN:-0.02}"
THREADS="${THREADS:-0}"
SENS="${SENS:-1}"
SKIP_BUILD="${SKIP_BUILD:-0}"
PY="${PY:-$HOME/miniconda3/envs/yolov8/bin/python}"

DATA="$HERE/data"
FIGS="$HERE/figs"
mkdir -p "$DATA" "$FIGS"

echo "== C4.1 =========================================================="
echo " workspace : $WS"
echo " samples   : $SAMPLES   d_min: $D_MIN m   threads: $THREADS"
echo " python    : $PY"
echo "=================================================================="

if [ "$SKIP_BUILD" != "1" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)" && conda activate yolov8
  fi
  ( cd "$WS" && colcon build --merge-install --symlink-install \
      --cmake-args "-DPython3_EXECUTABLE=$(which python3)" \
      --packages-select "$PKG_NAME" )
fi

# colcon's setup.bash reads COLCON_TRACE unguarded, which trips `set -u`.
set +u
# shellcheck disable=SC1091
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
EXE="$WS/build/$PKG_NAME/workspace_sample"
[ -x "$EXE" ] || { echo "missing $EXE -- run colcon build first" >&2; exit 1; }

run() {  # run <prefix> <extra args...>
  local prefix="$1"; shift
  echo
  echo "--- sampling -> $prefix"
  "$EXE" --samples "$SAMPLES" --threads "$THREADS" --out "$prefix" "$@"
}

echo
echo "### main run (d_min=$D_MIN)"
run "$DATA/workspace_dual_arm" --d-min "$D_MIN"

echo
echo "### plotting"
"$PY" "$HERE/plot_workspace.py" --prefix "$DATA/workspace_dual_arm" --out-dir "$FIGS"

if [ "$SENS" = "1" ]; then
  echo
  echo "### sensitivity: numerical-rank tolerance"
  for tol in 1e-4 1e-8; do
    run "$DATA/ws_ranktol_$tol" --d-min "$D_MIN" --rank-tol "$tol" >/dev/null
    echo "  rank_tol=$tol -> $(grep -o '"rank_defect": [0-9]*' "$DATA/ws_ranktol_$tol.json")"
  done

  echo
  echo "### sensitivity: self-collision threshold"
  run "$DATA/ws_dmin_0.05" --d-min 0.05 >/dev/null
  echo "  0.05 m -> $(grep -o '"accepted": [0-9]*' "$DATA/ws_dmin_0.05.json")"

  echo
  echo "### sensitivity: ground plane (how much of the raw workspace is below z=0)"
  run "$DATA/ws_nofloor" --d-min "$D_MIN" --no-z-floor >/dev/null
  echo "  no floor -> $(grep -o '"accepted": [0-9]*' "$DATA/ws_nofloor.json")"

  "$PY" - "$DATA" "$FIGS" <<'PYEOF'
import json, os, sys
data, figs = sys.argv[1], sys.argv[2]
out = {}
for tag, path in [("rank_tol_1e-4", "ws_ranktol_1e-4"), ("rank_tol_1e-6", "workspace_dual_arm"),
                  ("rank_tol_1e-8", "ws_ranktol_1e-8"), ("d_min_0.05", "ws_dmin_0.05"),
                  ("z_floor_disabled", "ws_nofloor")]:
    p = os.path.join(data, path + ".json")
    if not os.path.exists(p):
        continue
    s = json.load(open(p))["stats"]
    out[tag] = {"accepted": s["accepted"], "rank_defect": s["rank_defect"],
                "acceptance_rate": s["acceptance_rate"],
                "rejected_floor": s.get("rejected_floor", 0)}
sp = os.path.join(figs, "sensitivity.json")
json.dump(out, open(sp, "w"), indent=2)
print("wrote", sp)
print(json.dumps(out, indent=2))
PYEOF
fi

echo
echo "=== C4.1 done ==="
ls -la "$FIGS"
