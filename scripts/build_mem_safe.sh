#!/usr/bin/env bash
# Memory-safe colcon build wrapper for this project.
#
# Why this exists
# ---------------
# The dev box is a WSL2 distro with ~7.6 GiB RAM and 2 GiB swap, but it reports
# 16 CPUs.  colcon/make therefore default to -j16, and one Eigen-heavy
# translation unit in dual_arm_qp_core peaks at ~1.16 GiB RSS
# (measured: `dual_arm_qp.cpp`, -O3, 32 s CPU).  16 of those at once demand
# ~18 GiB.  The kernel OOM killer then picks the largest *resident* process,
# which is the agent itself (700 MB - 1.3 GiB), not the short-lived cc1plus --
# that is why `dsh` kept getting killed:
#
#   13:37:51  Out of memory: Killed process 2235  (MainThread) anon-rss 1.29 GB
#   14:14:26  Out of memory: Killed process 23040 (MainThread) anon-rss 944 MB
#   17:22:42  Out of memory: Killed process 8416  (MainThread) anon-rss 769 MB
#
# Usage
# -----
#   bash scripts/build_mem_safe.sh                       # whole workspace, -j2
#   JOBS=1 bash scripts/build_mem_safe.sh                # even gentler
#   PKGS="dual_arm_qp_core dual_arm_qp_controller" \
#     bash scripts/build_mem_safe.sh                     # narrow rebuild (preferred)
#   bash scripts/build_mem_safe.sh --packages-select foo # any extra colcon args
#
# Environment
#   JOBS   parallel compile jobs (default 2).  Peak RSS ~= JOBS * 1.2 GiB.
#   PKGS   space separated packages -> --packages-select
#   UPTO   space separated packages -> --packages-up-to (use on a fresh clone)
#   SKIP_CONDA=1   do not touch conda (assumes python3 already has empy)
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

JOBS="${JOBS:-2}"
case "$JOBS" in
  ''|*[!0-9]*) echo "ERROR: JOBS must be a positive integer (got '$JOBS')"; exit 2 ;;
esac

# rosidl_generate_interfaces needs empy, which lives in the project's conda env.
# Without it the controller build dies with
#   ModuleNotFoundError: No module named 'em'
if [ -z "${SKIP_CONDA:-}" ] && command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)" >/dev/null 2>&1 || true
  conda activate yolov8 >/dev/null 2>&1 || true
fi
if ! python3 -c "import em" >/dev/null 2>&1; then
  echo "ERROR: $(command -v python3) cannot import 'em' (empy); the rosidl generator will fail."
  echo "       Activate the project env (conda activate yolov8) or set SKIP_CONDA=1 with a proper PATH."
  exit 2
fi

export MAKEFLAGS="-j${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}"

PKG_ARGS=()
if [ -n "${UPTO:-}" ]; then
  # Build these packages *and everything they depend on* (unlike PKGS, which
  # selects exactly what is named).  Use this on a fresh clone.
  PKG_ARGS=(--packages-up-to ${UPTO})
elif [ -n "${PKGS:-}" ]; then
  # shellcheck disable=SC2206  # word splitting is intended here
  PKG_ARGS=(--packages-select ${PKGS})
fi

avail_mb="$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"
peak_mb=$(( JOBS * 1200 ))
echo "== memory-safe build =="
echo "   jobs          : $JOBS (MAKEFLAGS=$MAKEFLAGS, CMAKE_BUILD_PARALLEL_LEVEL=$CMAKE_BUILD_PARALLEL_LEVEL)"
echo "   python3       : $(command -v python3)"
echo "   MemAvailable  : ${avail_mb} MiB"
echo "   estimated peak: ~${peak_mb} MiB (JOBS x ~1.2 GiB)"
if [ "$avail_mb" -lt "$peak_mb" ]; then
  echo "   WARNING: available memory is below the estimate -- consider JOBS=1"
fi
echo

colcon build \
  --merge-install \
  --symlink-install \
  --parallel-workers "$JOBS" \
  "${PKG_ARGS[@]}" \
  "$@" \
  --cmake-args "-DPython3_EXECUTABLE=$(command -v python3)"

echo
echo "== done: $(date -u +%Y-%m-%dT%H:%M:%SZ) =="
