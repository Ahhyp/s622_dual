#!/usr/bin/env bash
# C4.3 S4 W0 -- reproducible form of acceptance clause W0-2.
#
# Proves the migration dual_arm_transport_mock -> dual_arm_object_task is
# numerically inert: the SAME dump source is compiled twice, once with the
# pre-migration sources and once with the post-migration sources, and the two
# outputs are compared with cmp.  Every number is printed with %a, so this is a
# bitwise comparison and no tolerance is involved.
#
# Both sides compile the moved translation units directly (they are only two
# files) instead of linking a prebuilt library, so the check does not depend on
# which package happens to be installed.
#
# Usage:
#   git worktree add /tmp/w0_before 627a761      # parent of the W0 commit
#   W0_BEFORE_ROOT=/tmp/w0_before tools/w0_migration_check.sh
#
# Without W0_BEFORE_ROOT only the after side is built and hashed, which is still
# useful as a regression fingerprint.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(cd "${here}/.." && pwd)"
after_root="$(cd "${pkg}/../.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

flags=(-std=c++17 -O3 -Wall -Wextra)
inc_common=(-I "${after_root}/src/dual_arm_kinematics/include"
            -I "${after_root}/src/fairino_planning_core/include"
            -I /usr/include/eigen3)
libs=(-L "${after_root}/install/lib" -ldual_arm_kinematics -lyaml-cpp
      -Wl,-rpath,"${after_root}/install/lib")

echo "[w0] work dir: ${work}"

# ---- after: dual_arm_object_task ----
g++ "${flags[@]}" -DC43_DUMP_NEW_LAYOUT \
    "${here}/dump_anchored_task_samples.cpp" \
    "${pkg}/src/anchored_object_task.cpp" \
    "${pkg}/src/trajectory.cpp" \
    -o "${work}/dump_after" \
    -I "${pkg}/include" -I "${after_root}/src/dual_arm_transport_mock/tools" \
    "${inc_common[@]}" "${libs[@]}"
LD_LIBRARY_PATH="${after_root}/install/lib:${LD_LIBRARY_PATH:-}" \
    "${work}/dump_after" > "${work}/after.txt"
echo "[w0] after : $(sha256sum "${work}/after.txt" | cut -d' ' -f1)  $(wc -l < "${work}/after.txt") lines"

if [[ -z "${W0_BEFORE_ROOT:-}" ]]; then
    echo "[w0] W0_BEFORE_ROOT not set: skipping the cmp (after-side fingerprint only)"
    cp "${work}/after.txt" "${pkg}/w0_after_dump.txt"
    exit 0
fi

# ---- before: dual_arm_transport_mock ----
mock="${W0_BEFORE_ROOT}/src/dual_arm_transport_mock"
g++ "${flags[@]}" -DC43_DUMP_OLD_LAYOUT \
    "${here}/dump_anchored_task_samples.cpp" \
    "${mock}/src/object_task.cpp" \
    "${mock}/src/trajectory.cpp" \
    -o "${work}/dump_before" \
    -I "${mock}/include" -I "${mock}/tools" \
    "${inc_common[@]}" "${libs[@]}"
LD_LIBRARY_PATH="${after_root}/install/lib:${LD_LIBRARY_PATH:-}" \
    "${work}/dump_before" > "${work}/before.txt"
echo "[w0] before: $(sha256sum "${work}/before.txt" | cut -d' ' -f1)  $(wc -l < "${work}/before.txt") lines"

if cmp -s "${work}/before.txt" "${work}/after.txt"; then
    echo "[w0] PASS: outputs are bit-identical"
else
    echo "[w0] FAIL: outputs differ"
    diff -u "${work}/before.txt" "${work}/after.txt" | head -20
    exit 1
fi
