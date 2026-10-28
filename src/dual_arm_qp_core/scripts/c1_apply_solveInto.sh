#!/usr/bin/env bash
# C4.3 S4 / W2-C1: apply the approved solveInto patch and rebuild (D1-A only).
#
# Run this ONLY after D1-A is signed.  It refuses to run on a drifted tree and
# it never touches the G-14 baseline by itself: at the end it prints the new
# libdual_arm_qp_core.so hash for a human to record in the Gate matrix.
#
# Usage: bash src/dual_arm_qp_core/scripts/c1_apply_solveInto.sh
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

# `rosidl_generate_interfaces` needs `em` (empy), which ships in the project's
# conda env.  Building the controller without it dies with
#   ModuleNotFoundError: No module named 'em'
if [ -z "${C1_SKIP_CONDA:-}" ] && command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)" >/dev/null 2>&1 && conda activate yolov8 >/dev/null 2>&1 || true
fi
if ! python3 -c "import em" >/dev/null 2>&1; then
  echo "ABORT: $(command -v python3) cannot import 'em' (empy)."
  echo "        Activate the project env first (conda activate yolov8), then re-run."
  exit 4
fi
echo "python3=$(command -v python3) (empy ok)"

PATCH="docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-W3_C1_solveInto.patch"
SNAPSHOT="docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-W3_C1_集成前快照"
EVIDENCE="${C1_EVIDENCE_DIR:-/tmp/c43s4_c1_integration}"
FILES=(
  src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp
  src/dual_arm_qp_core/include/dual_arm_qp_core/osqp_solver.hpp
  src/dual_arm_qp_core/src/osqp_solver.cpp
  src/dual_arm_qp_core/src/dual_arm_qp.cpp
)

mkdir -p "$EVIDENCE"

# --resume: the patch is already applied (e.g. a later step failed); verify the
# tree is at the post-patch state instead of the pre-integration snapshot.
RESUME=0
[ "${1:-}" = "--resume" ] && RESUME=1

echo "== 0/8 preflight =="
if [ "$RESUME" = "1" ]; then
  for f in "${FILES[@]}"; do
    got="$(sha256sum "$f" | cut -d' ' -f1)"
    if ! grep -q " $f\$" "$EVIDENCE/post_patch_sha256.txt" 2>/dev/null || \
       ! grep -q "^$got " "$EVIDENCE/post_patch_sha256.txt"; then
      echo "ABORT: $f ($got) is not the post-patch content recorded in"
      echo "        $EVIDENCE/post_patch_sha256.txt; cannot resume."
      exit 4
    fi
  done
  echo "OK: resuming -- the four files match the recorded post-patch hashes"
else
  for f in "${FILES[@]}"; do
    want="$(grep " $f\$" "$SNAPSHOT/SHA256SUMS.pre" | cut -d' ' -f1)"
    got="$(sha256sum "$f" | cut -d' ' -f1)"
    if [ "$want" != "$got" ]; then
      echo "ABORT: $f has drifted since the snapshot (want $want, got $got)"
      echo "        restore it first with: bash src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh --restore-only"
      exit 4
    fi
  done
  echo "OK: the four files still match the pre-integration snapshot"

  git apply --check "$PATCH"
  echo "OK: patch applies cleanly"
fi
if [ "$(strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate)" != "0" ]; then
  echo "ABORT: the production controller library is not the OFF build"; exit 4
fi
echo "OK: production library is hooks=OFF (0 injection symbols)"

if [ "$RESUME" = "1" ]; then
  echo "== 1/8 apply: SKIPPED (--resume) =="
else
  echo "== 1/8 apply =="
  git apply "$PATCH"
  sha256sum "${FILES[@]}" | tee "$EVIDENCE/post_patch_sha256.txt"
fi

echo "== 2/8 build dual_arm_transport_mock (legacy golden tool) =="
if [ "$RESUME" = "1" ] && grep -q "Finished   <<< dual_arm_transport_mock" "$EVIDENCE/build_transport_mock.log" 2>/dev/null; then
  echo "SKIPPED (--resume): already finished in $EVIDENCE/build_transport_mock.log"
else
  colcon build --merge-install --symlink-install --packages-select dual_arm_transport_mock \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)" 2>&1 | tee "$EVIDENCE/build_transport_mock.log"
fi

echo "== 3/8 legacy golden must be regenerated if the dq changed =="
echo "   (generator: dual_arm_transport_mock/tools/gen_legacy_golden.cpp;"
echo "    if 'git status' shows the golden changed, that change belongs in this commit)"

echo "== 4/8 build dual_arm_qp_core + dual_arm_qp_controller =="
colcon build --merge-install --symlink-install \
  --packages-select dual_arm_qp_core dual_arm_qp_controller \
  --cmake-args "-DPython3_EXECUTABLE=$(which python3)" 2>&1 | tee "$EVIDENCE/build_core_controller.log"

echo "== 5/8 regression (controller + executor) =="
bash src/dual_arm_qp_controller/scripts/run_c43s4_tests.sh 2>&1 | tee "$EVIDENCE/tests.log"
tail -5 "$EVIDENCE/tests.log"

echo "== 6/8 record the new core hash =="
NEW_HASH="$(sha256sum install/lib/libdual_arm_qp_core.so | cut -d' ' -f1)"
echo "$NEW_HASH" | tee "$EVIDENCE/libdual_arm_qp_core.sha256"
echo "   OLD baseline (historical, must stay in the doc): 329a3bb35c286400c8438aa4c7e00cbe"
echo "   NEW baseline (colcon build, record this in G-14): $NEW_HASH"

echo "== 7/8 production library must remain hooks=OFF =="
test "$(strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate)" = "0" && echo "OK: 0 injection symbols"

echo "== 8/8 done =="
echo "Next (manual, requires D1-A approval on record):"
echo "  update the G-14 row in C4.3_S4_W2-W3双轨门禁矩阵.md:"
echo "    old 329a3bb35c286400c8438aa4c7e00cbe -> $NEW_HASH"
echo "Rollback (no git restore needed): bash src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh"
