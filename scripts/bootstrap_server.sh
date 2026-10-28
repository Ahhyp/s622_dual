#!/usr/bin/env bash
# C4.3 S4 — first build + smoke test on a fresh Linux machine.
#
# Usage:
#   bash scripts/bootstrap_server.sh              # build everything, run the fast suites
#   bash scripts/bootstrap_server.sh --full       # + controller/transport ctest
#   bash scripts/bootstrap_server.sh --with-hooks # + a separate hooks=ON overlay build
#
# It refuses to guess: every prerequisite is checked and named.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

FULL=0
HOOKS=0
for a in "$@"; do
  case "$a" in
    --full) FULL=1 ;;
    --with-hooks) HOOKS=1 ;;
    *) echo "unknown argument: $a" >&2; exit 2 ;;
  esac
done

say() { printf '\n=== %s ===\n' "$*"; }
fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }

say "1. prerequisites"
[ -f /opt/ros/humble/setup.bash ] || fail "ROS 2 Humble not found at /opt/ros/humble (this project targets Humble)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for pkg in ros_gz_sim ros_gz_bridge controller_manager xacro joint_state_broadcaster; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fail "missing ROS package: $pkg (install ros-humble-$pkg / ros-humble-ros-gz)"
done
ign gazebo --versions 2>/dev/null | head -1 || fail "ign gazebo not found (Fortress 6.x is what W6-0 was measured against)"
command -v colcon >/dev/null || fail "colcon not found (python3 -m pip install colcon-common-extensions)"
[ -d /usr/include/eigen3 ] || fail "Eigen3 headers not found (apt install libeigen3-dev)"
[ -d /usr/include/osqp ] || fail "OSQP headers not found (apt install libosqp-dev or build from source)"
[ -d /usr/include/yaml-cpp ] || fail "yaml-cpp headers not found (apt install libyaml-cpp-dev)"

say "2. python (the ROS 2 interface generator needs empy; conda yolov8 is what this repo used)"
PY="${PYTHON_EXECUTABLE:-$(command -v python3)}"
"$PY" -c "import em" 2>/dev/null || fail "python3 cannot import 'em' (empy). Activate the interpreter that has em, or export PYTHON_EXECUTABLE=/path/to/python3"
echo "using python: $PY"

say "3. build (memory-safe: 2 parallel jobs; a single Eigen translation unit peaks at ~1.2 GB)"
JOBS="${JOBS:-4}" bash scripts/build_mem_safe.sh || fail "build failed"

say "4. invariants"
STRINGS=$(strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate || true)
echo "test_fail_readstate symbols in the production library: $STRINGS (must be 0)"
[ "$STRINGS" = "0" ] || fail "the production library carries test-injection symbols"
echo -n "libdual_arm_qp_core.so sha256: "; sha256sum install/lib/libdual_arm_qp_core.so | cut -d' ' -f1
echo -n "frozen S3 fixture sha256:       "; sha256sum src/dual_arm_qp_core/test/fixtures/s3_ff_1_dq.csv | cut -d' ' -f1
echo -n "D2-B baseline sha256:           "; sha256sum src/dual_arm_qp_core/test/fixtures/c3_d2b/c3_d2b_candidate_baseline.csv | cut -d' ' -f1

say "5. fast suites"
# shellcheck disable=SC1091
source install/setup.bash
export LD_LIBRARY_PATH="$REPO_ROOT/install/lib:${LD_LIBRARY_PATH:-}"
C43_W3R6_ARTIFACT_DIR="${C43_W3R6_ARTIFACT_DIR:-/tmp/c43_s4_w3r6_dry_run}" \
  colcon test --merge-install --packages-select dual_arm_transport_executor \
  --test-result-base build/dual_arm_transport_executor --event-handlers console_direct- || true
colcon test-result --test-result-base build/dual_arm_transport_executor --verbose | tail -5

say "6. ledger + D2-B gate"
python3 src/dual_arm_transport_executor/scripts/check_acceptance_yaml.py || fail "acceptance ledger check failed"
python3 src/dual_arm_qp_core/scripts/c3_d2b_gate.py \
  --baseline src/dual_arm_qp_core/test/fixtures/c3_d2b/c3_d2b_candidate_baseline.csv \
  --rerun    src/dual_arm_qp_core/test/fixtures/c3_d2b/c3_d2b_candidate_baseline.rerun.csv \
  --provenance src/dual_arm_qp_core/test/fixtures/c3_d2b/PROVENANCE.md \
  --lib      install/lib/libdual_arm_qp_core.so \
  --admm     src/dual_arm_qp_core/test/fixtures/c3_d2b/c3_d2b_admm.csv || fail "D2-B gate failed"

if [ "$FULL" = "1" ]; then
  say "7. full ctest (controller + executor)"
  colcon test --merge-install --packages-select dual_arm_transport_executor dual_arm_qp_controller \
    --test-result-base build --event-handlers console_direct- || true
  colcon test-result --verbose | tail -10
fi

if [ "$HOOKS" = "1" ]; then
  say "8. hooks=ON overlay (T_H17/T_H17a need the injected fault; they SKIP in the production build)"
  colcon build --merge-install --symlink-install --build-base build_hooks_on --install-base install_hooks_on \
    --packages-select dual_arm_qp_controller \
    --cmake-args "-DPython3_EXECUTABLE=$PY" -DC43_ENABLE_TEST_HOOKS=ON || fail "hooks=ON build failed"
  echo "NOTE: the gtest forks 'ros2 launch', so it must also see the ON overlay:"
  echo "      source install_hooks_on/setup.bash && export LD_LIBRARY_PATH=$REPO_ROOT/install_hooks_on/lib:\$LD_LIBRARY_PATH"
fi

say "done"
echo "Next: bash src/dual_arm_qp_controller/scripts/audit_native_rt_verdict.py --help   # C6 gate audit"
echo "On a real RT machine: bash src/dual_arm_qp_controller/scripts/run_native_rt_acceptance.sh"
