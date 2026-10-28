#!/usr/bin/env bash
# C4.3 S4 -- reproducible test entry point (W2/W3 final acceptance round).
#
# Why this exists (P1-1/P1-2): running `colcon test` from a shell that only
# sourced /opt/ros/humble makes the bench-launching gtests fail with
#   Package 'dual_arm_qp_controller' not found: searching: ['/opt/ros/humble']
# because the launch child inherits an overlay-less environment, and the
# executor tests additionally need the install lib dir on LD_LIBRARY_PATH
# (libdual_arm_collision -> libfairino_planning_core is a transitive dep that
# RUNPATH alone does not resolve).  Both are environment facts, not code bugs;
# this script makes the environment explicit instead of relying on the caller.
#
# Usage (from a clean shell, no prior sourcing required):
#   ./src/dual_arm_qp_controller/scripts/run_c43s4_tests.sh                  # both packages, hooks OFF
#   ./src/dual_arm_qp_controller/scripts/run_c43s4_tests.sh --hooks-on       # T-H17/T_H17a really run
#   ./src/dual_arm_qp_controller/scripts/run_c43s4_tests.sh dual_arm_qp_core # one package
#
# --hooks-on expects a second, hook-enabled build produced with:
#   colcon build --merge-install --symlink-install --packages-select dual_arm_qp_controller \
#     --build-base build_hooks_on --install-base install_hooks_on \
#     --cmake-args "-DPython3_EXECUTABLE=$(which python3)" -DC43_ENABLE_TEST_HOOKS=ON
# The prefix must be *sourced* and its lib dir placed first: the gtests launch
# the controller through `ros2 launch`, so an ON binary alone is not enough --
# without the overlay the launch still loads the OFF library and the injection
# silently never fires (observed as "injected ERROR was not taken").
set -eo pipefail   # NOTE: no `set -u` -- ROS's setup.bash reads unset vars

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

HOOKS_ON=0
PACKAGES=()
for arg in "$@"; do
  case "$arg" in
    --hooks-on) HOOKS_ON=1 ;;
    *) PACKAGES+=("$arg") ;;
  esac
done
if [ ${#PACKAGES[@]} -eq 0 ]; then
  PACKAGES=(dual_arm_qp_controller dual_arm_transport_executor)
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
export LD_LIBRARY_PATH="$REPO_ROOT/install/lib:${LD_LIBRARY_PATH:-}"

if [ "$HOOKS_ON" = "1" ]; then
  if [ ! -f install_hooks_on/setup.bash ]; then
    echo "[c43s4] ERROR: --hooks-on needs install_hooks_on/ (see header for the build command)" >&2
    exit 2
  fi
  # shellcheck disable=SC1091
  source install_hooks_on/setup.bash
  export LD_LIBRARY_PATH="$REPO_ROOT/install_hooks_on/lib:$LD_LIBRARY_PATH"
  echo "[c43s4] hooks=ON: $(strings install_hooks_on/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate) injection symbol(s) in the overlay"
fi

export C43_W3R6_ARTIFACT_DIR="${C43_W3R6_ARTIFACT_DIR:-/tmp/c43_s4_w3r6_dry_run}"

echo "[c43s4] packages: ${PACKAGES[*]}"
echo "[c43s4] AMENT_PREFIX_PATH has install: $(echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | grep -c "^$REPO_ROOT/install")"
echo "[c43s4] LD_LIBRARY_PATH head: $(echo "$LD_LIBRARY_PATH" | cut -d: -f1)"
echo "[c43s4] artifact dir: $C43_W3R6_ARTIFACT_DIR"

# Serial: several controller tests fork a mock bench and bind the same DDS
# domain / services; running them concurrently produces "bench never came up"
# flakes that are environment contention, not regressions.
colcon test --merge-install --packages-select "${PACKAGES[@]}" \
  --executor sequential --event-handlers console_direct- || true
for pkg in "${PACKAGES[@]}"; do
  echo "[c43s4] ---- $pkg ----"
  # Scope the summary to this package: `--test-result-base build` would also
  # pick up unrelated packages' stale results and report their failures here.
  colcon test-result --test-result-base "build/$pkg" --verbose | tail -25
done
