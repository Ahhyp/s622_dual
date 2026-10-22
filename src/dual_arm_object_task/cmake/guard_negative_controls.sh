#!/usr/bin/env bash
# C4.3 S4 W0 -- negative controls for the W0-1 acceptance guard.
#
# The guard (check_single_object_task_definition.cmake) only earns trust if it is
# shown to FAIL on the situations it is supposed to catch, and to stay quiet on
# legitimate code.  This script builds throw-away trees under a temp directory and
# prints the exit code of every case.  It never touches the real source tree.
#
# Expected: P1..P2 -> 0, N1..N6 -> non-zero.
# Usage: bash cmake/guard_negative_controls.sh [--keep]
set -u

GUARD="$(cd "$(dirname "$0")" && pwd)/check_single_object_task_definition.cmake"
SRC_REAL="$(cd "$(dirname "$0")/../.." && pwd)"

if [ ! -f "$GUARD" ]; then
    echo "guard script not found: $GUARD" >&2
    exit 2
fi

KEEP=0
for arg in "$@"; do
    [ "$arg" = "--keep" ] && KEEP=1
done

ROOT="$(mktemp -d /tmp/c43_w0_guard_controls.XXXXXX)"
trap '[ "$KEEP" = "1" ] || rm -rf "$ROOT"' EXIT

FAILED=0
run() { # <case> <src-root> <expected: pass|fail>
    local case_name="$1" tree="$2" expected="$3" rc
    cmake -DSRC_ROOT="$tree" -P "$GUARD" >"$ROOT/$case_name.log" 2>&1
    rc=$?
    local verdict="OK"
    if [ "$expected" = "pass" ] && [ "$rc" -ne 0 ]; then verdict="UNEXPECTED-FAILURE"; fi
    if [ "$expected" = "fail" ] && [ "$rc" -eq 0 ]; then verdict="UNEXPECTED-PASS"; fi
    [ "$verdict" = "OK" ] || FAILED=1
    printf '%-28s expected=%-4s exit=%d  %s\n' "$case_name" "$expected" "$rc" "$verdict"
}

# ---- minimal tree shaped like the real one ------------------------------------
build_tree() {
    local T="$1"
    rm -rf "$T"
    mkdir -p "$T/dual_arm_object_task/include/dual_arm_object_task"
    cat >"$T/dual_arm_object_task/include/dual_arm_object_task/anchored_object_task.hpp" <<'EOF'
#pragma once
namespace dual_arm_object_task {
class AnchoredObjectTaskBuilder {
 public:
  int x = 0;
};
struct AnchoredObjectTask {
  int y = 0;
};
}  // namespace dual_arm_object_task
EOF
    mkdir -p "$T/dual_arm_qp_core/include/dual_arm_qp_core"
    cat >"$T/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp" <<'EOF'
#pragma once
#include "dual_arm_object_task/anchored_object_task.hpp"
namespace dual_arm_qp {
// The object row comes from AnchoredObjectTaskBuilder; never call
// anchoredObjectPose( or decoupledPoseError( here.
class AnchoredObjectTaskBuilder;   // forward declaration only
struct AnchoredObjectTask;         // forward declaration only
struct ObjectTaskRow { int J; };
}  // namespace dual_arm_qp
EOF
    mkdir -p "$T/dual_arm_kinematics/include/dual_arm_kinematics"
    cat >"$T/dual_arm_kinematics/include/dual_arm_kinematics/object_frame.hpp" <<'EOF'
#pragma once
namespace dual_arm {
inline void anchoredObjectPose(double) {}
inline void decoupledPoseError(double) {}
}  // namespace dual_arm
EOF
}

echo "== positive controls (must pass) =="
run P1_real_source_tree "$SRC_REAL" pass

build_tree "$ROOT/p2"
run P2_synthetic_clean_tree "$ROOT/p2" pass

echo "== negative controls (must fail) =="
# N1: a second builder definition in another package -- the hole of the first version
build_tree "$ROOT/n1"
mkdir -p "$ROOT/n1/other_pkg/include/other_pkg"
printf 'class AnchoredObjectTaskBuilder { };\n' >"$ROOT/n1/other_pkg/include/other_pkg/copy.hpp"
run N1_second_builder_other_pkg "$ROOT/n1" fail

# N2: a second copy of the object-task math, as a real call (not a declaration)
build_tree "$ROOT/n2"
mkdir -p "$ROOT/n2/other_pkg/src"
cat >"$ROOT/n2/other_pkg/src/derive.cpp" <<'EOF'
void f() {
  auto J = dual_arm::anchoredObjectErrorJacobian(Jb, G, T_hat, T_d);
}
EOF
run N2_second_math_other_pkg "$ROOT/n2" fail

# N3: vacuous scan
mkdir -p "$ROOT/n3"
run N3_empty_tree "$ROOT/n3" fail

# N4: canonical definition file missing
build_tree "$ROOT/n4"
rm "$ROOT/n4/dual_arm_object_task/include/dual_arm_object_task/anchored_object_task.hpp"
run N4_canonical_header_missing "$ROOT/n4" fail

# N6: two definitions inside the canonical package
build_tree "$ROOT/n6"
mkdir -p "$ROOT/n6/dual_arm_object_task/src"
printf 'class AnchoredObjectTaskBuilder { };\n' >"$ROOT/n6/dual_arm_object_task/src/dup.cpp"
run N6_duplicate_inside_canonical "$ROOT/n6" fail

echo "== non-tripping control (must pass) =="
# N5: comments, includes, forward declarations and test code only -- must NOT trip
build_tree "$ROOT/n5"
mkdir -p "$ROOT/n5/dual_arm_object_task/test"
cat >"$ROOT/n5/dual_arm_object_task/test/test_task.cpp" <<'EOF'
#include "dual_arm_object_task/anchored_object_task.hpp"
// test only: builder.build();  AnchoredObjectTaskBuilder b;
void g() {
  dual_arm_object_task::AnchoredObjectTask t;
  dual_arm_object_task::AnchoredObjectTaskBuilder b;
  (void)t; (void)b;
}
EOF
run N5_legit_references_only "$ROOT/n5" pass

echo
if [ "$FAILED" -ne 0 ]; then
    echo "RESULT: FAILED -- at least one control did not behave as specified"
    exit 1
fi
echo "RESULT: OK -- all controls behaved as specified"
if [ "$KEEP" = "1" ]; then
    echo "logs kept in: $ROOT"
fi
exit 0
