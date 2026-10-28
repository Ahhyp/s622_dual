#!/usr/bin/env bash
# C4.3 S4 / W2-C3 -- NEGATIVE tests for the D2-B gate.
#
# Mandatory D2-B condition #5: the gate must be shown to FAIL when something is
# actually wrong.  A gate that only ever passes is not a gate.  Each case below
# corrupts exactly one aspect of the input and asserts the expected exit code and
# decision string.
#
#   00 control      unmodified fixtures                -> exit 0  D2B_PASS
#   01 one dq bit   one mantissa bit flipped in rerun  -> exit 3  D2B_FAIL
#   02 missing      baseline file absent               -> exit 2  INPUT_ERROR
#   03 schema       a dq column renamed                -> exit 2  INPUT_ERROR
#   04 short run    rerun truncated to 637 rows        -> exit 2  INPUT_ERROR
#   05 provenance   provenance claims another library  -> exit 2  INPUT_ERROR
#   06 no admm      cross-algorithm evidence absent    -> exit 2  (argparse/tool error)
#
# Usage: bash src/dual_arm_qp_core/scripts/run_c3_d2b_negative.sh [--keep]
set -o pipefail   # NOT -u: ROS setup scripts are not nounset-clean

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FIX="$REPO_ROOT/src/dual_arm_qp_core/test/fixtures/c3_d2b"
GATE="$REPO_ROOT/src/dual_arm_qp_core/scripts/c3_d2b_gate.py"
LIB="$REPO_ROOT/install/lib/libdual_arm_qp_core.so"
PY="${PYTHON:-python3}"
WORK="$(mktemp -d /tmp/c3d2b_neg.XXXXXX)"
KEEP=0
[ "${1:-}" = "--keep" ] && KEEP=1

fail=0
run() { # run <name> <expected_exit> <expected_decision> <args...>
  local name="$1" want_code="$2" want_decision="$3"; shift 3
  local out code
  out="$("$PY" "$GATE" "$@" 2>&1)"; code=$?
  if [ "$code" -ne "$want_code" ]; then
    echo "  [FAIL] $name: exit $code (expected $want_code)"; echo "$out" | sed 's/^/         /'
    fail=$((fail + 1)); return
  fi
  if ! echo "$out" | grep -q "C3_D2B: $want_decision"; then
    echo "  [FAIL] $name: decision not $want_decision"; echo "$out" | sed 's/^/         /'
    fail=$((fail + 1)); return
  fi
  echo "  [ ok ] $name: exit $code, C3_D2B: $want_decision"
}

echo "== C3 D2-B negative gate tests (fixtures: $FIX) =="

# --- 01: flip one mantissa bit of one dq value -------------------------------
"$PY" - "$FIX/c3_d2b_candidate_baseline.rerun.csv" "$WORK/flipped.csv" <<'EOF'
import sys
src, dst = sys.argv[1], sys.argv[2]
lines = open(src).read().splitlines()
head = lines[0].split(",")
col = head.index("dq0")
parts = lines[1].split(",")
v = float(parts[col])
parts[col] = repr(v + 1e-18 * (1 if v >= 0 else -1))   # ~ one lsb at 1e-14
lines[1] = ",".join(parts)
open(dst, "w").write("\n".join(lines) + "\n")
EOF
run "01 one dq bit flipped"          3 D2B_FAIL \
  --baseline "$FIX/c3_d2b_candidate_baseline.csv" --rerun "$WORK/flipped.csv" \
  --admm "$FIX/c3_d2b_admm.csv"

# --- 02: missing baseline ----------------------------------------------------
run "02 missing baseline"            2 INPUT_ERROR \
  --baseline "$WORK/nope.csv" --rerun "$FIX/c3_d2b_candidate_baseline.rerun.csv" \
  --admm "$FIX/c3_d2b_admm.csv"

# --- 03: schema changed ------------------------------------------------------
sed '1s/\bdq3\b/dq3_typo/' "$FIX/c3_d2b_candidate_baseline.rerun.csv" > "$WORK/schema.csv"
run "03 schema changed"              2 INPUT_ERROR \
  --baseline "$FIX/c3_d2b_candidate_baseline.csv" --rerun "$WORK/schema.csv" \
  --admm "$FIX/c3_d2b_admm.csv"

# --- 04: truncated run -------------------------------------------------------
head -638 "$FIX/c3_d2b_candidate_baseline.rerun.csv" > "$WORK/short.csv"
run "04 truncated run"               2 INPUT_ERROR \
  --baseline "$FIX/c3_d2b_candidate_baseline.csv" --rerun "$WORK/short.csv" \
  --admm "$FIX/c3_d2b_admm.csv"

# --- 05: provenance names a different library --------------------------------
sed 's/`ff95750afe76b49b7738c35eb1264f94791f81004513510e2c8ca939e90a4636`/`0000000000000000000000000000000000000000000000000000000000000000`/' \
  "$FIX/PROVENANCE.md" > "$WORK/PROVENANCE_wrong.md"
run "05 provenance mismatch"         2 INPUT_ERROR \
  --baseline "$FIX/c3_d2b_candidate_baseline.csv" \
  --rerun "$FIX/c3_d2b_candidate_baseline.rerun.csv" --admm "$FIX/c3_d2b_admm.csv" \
  --provenance "$WORK/PROVENANCE_wrong.md" --lib "$LIB"

# --- 06: cross-algorithm evidence omitted (argparse must refuse) -------------
out="$("$PY" "$GATE" --baseline "$FIX/c3_d2b_candidate_baseline.csv" \
        --rerun "$FIX/c3_d2b_candidate_baseline.rerun.csv" 2>&1)"; code=$?
if [ "$code" -eq 2 ] && echo "$out" | grep -q "admm"; then
  echo "  [ ok ] 06 admm evidence omitted: exit 2 (argparse refuses)"
else
  echo "  [FAIL] 06 admm evidence omitted: exit $code"; echo "$out" | sed 's/^/         /'
  fail=$((fail + 1))
fi

# --- 00: control (must pass, with provenance binding) ------------------------
run "00 control"                     0 D2B_PASS \
  --baseline "$FIX/c3_d2b_candidate_baseline.csv" \
  --rerun "$FIX/c3_d2b_candidate_baseline.rerun.csv" --admm "$FIX/c3_d2b_admm.csv" \
  --provenance "$FIX/PROVENANCE.md" --lib "$LIB"

rm -rf "$WORK"
if [ "$fail" -ne 0 ]; then
  echo "NEGATIVE TESTS: $fail case(s) did not behave as expected"
  exit 1
fi
echo "NEGATIVE TESTS: all 7 cases behaved as required (gate is fail-closed)"
