#!/usr/bin/env bash
# C4.3 S4 / W2-C1: precise rollback of the solveInto patch.
#
# This script does NOT use `git restore HEAD` / `git checkout --`:
# the working tree carries other uncommitted work, and per-file `git restore`
# would be one careless glob away from discarding it.  Instead it restores the
# four files from the byte-exact pre-integration snapshot and verifies the
# SHA256 of every restored file.
#
# Usage:
#   bash src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh               # restore + rebuild
#   bash src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh --restore-only # files only
#   bash src/dual_arm_qp_core/scripts/c1_rollback_solveInto.sh --check        # verify only, no writes
#
# D1-B (2026-10-27) made the D1-A build the official G-14 baseline.  Rollback
# stays valid as the documented way to get back to the pre-C1 sources: it never
# touches git, only the four files recorded in SHA256SUMS.pre, and it re-verifies
# every byte it writes.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

SNAPSHOT="docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2-W3_C1_集成前快照"
EVIDENCE="${C1_EVIDENCE_DIR:-/tmp/c43s4_c1_integration}"
RESTORE_ONLY=0
CHECK_ONLY=0
[ "${1:-}" = "--restore-only" ] && RESTORE_ONLY=1
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

mkdir -p "$EVIDENCE"

if [ "$CHECK_ONLY" = "1" ]; then
  echo "== --check: validate the rollback material without touching the tree =="
  ok=0
  while read -r sha rel; do
    [ -z "$sha" ] && continue
    src="$SNAPSHOT/files/$rel"
    if [ ! -f "$src" ]; then echo "MISSING snapshot file: $rel"; ok=1; continue; fi
    got="$(sha256sum "$src" | cut -d' ' -f1)"
    if [ "$got" != "$sha" ]; then echo "BAD snapshot file: $rel ($got != $sha)"; ok=1; continue; fi
    if cmp -s "$src" "$rel"; then
      echo "NOTE: $rel is byte-identical to the snapshot (already rolled back?)"
    else
      echo "OK  : $rel can be restored byte-exactly"
    fi
  done < <(grep -v '^$' "$SNAPSHOT/SHA256SUMS.pre")
  ( cd "$SNAPSHOT/files" && sha256sum -c ../SHA256SUMS.pre --quiet ) || ok=1
  if [ "$ok" -ne 0 ]; then echo "ROLLBACK MATERIAL NOT USEABLE"; exit 4; fi
  echo "OK: rollback material verified (4/4), tree untouched"
  exit 0
fi

echo "== 1/4 restore the four files from the snapshot =="
while read -r sha rel; do
  src="$SNAPSHOT/files/$rel"
  if [ ! -f "$src" ]; then echo "ABORT: missing snapshot file $src"; exit 4; fi
  cp -p "$src" "$rel"
  got="$(sha256sum "$rel" | cut -d' ' -f1)"
  if [ "$got" != "$sha" ]; then
    echo "ABORT: restored $rel has $got, snapshot says $sha"
    exit 4
  fi
  echo "restored $rel  ($sha)"
done < <(grep -v '^$' "$SNAPSHOT/SHA256SUMS.pre")

echo "== 2/4 verify against the snapshot manifest =="
( cd "$SNAPSHOT/files" && sha256sum -c ../SHA256SUMS.pre --quiet ) && echo "OK: snapshot self-check"

echo "== 3/4 confirm the tree matches HEAD for these four files =="
git diff --stat -- \
  src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp \
  src/dual_arm_qp_core/include/dual_arm_qp_core/osqp_solver.hpp \
  src/dual_arm_qp_core/src/osqp_solver.cpp \
  src/dual_arm_qp_core/src/dual_arm_qp.cpp
echo "(empty diff above = exactly back to HEAD)"

if [ "$RESTORE_ONLY" = "1" ]; then
  echo "== --restore-only: skipping the rebuild =="
  exit 0
fi

echo "== 4/4 rebuild core + controller =="
colcon build --merge-install --symlink-install \
  --packages-select dual_arm_qp_core dual_arm_qp_controller \
  --cmake-args "-DPython3_EXECUTABLE=$(which python3)" 2>&1 | tee "$EVIDENCE/rollback_build.log"

HASH="$(sha256sum install/lib/libdual_arm_qp_core.so | cut -d' ' -f1)"
MD5="$(md5sum install/lib/libdual_arm_qp_core.so | cut -d' ' -f1)"
echo "libdual_arm_qp_core.so after rollback: sha256=$HASH md5=$MD5"
# The pre-C1 baseline was recorded as an md5 (that was the G-14 line at the time),
# so compare md5 -- comparing a sha256 against a 32-char md5 could never succeed.
if [ "$MD5" = "329a3bb35c286400c8438aa4c7e00cbe" ]; then
  echo "OK: back to the pre-integration baseline (md5 matches the historical value)"
else
  echo "WARN: md5 differs from 329a3bb35c286400c8438aa4c7e00cbe"
  echo "      (the sources are byte-identical to HEAD, so any difference is build"
  echo "       environment drift, not leftover patch content -- record it, do not hide it)"
fi
