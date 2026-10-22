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
#
# Evidence retention: set W0_EVIDENCE_DIR=<dir> to keep the raw before/after
# dumps, their SHA256 sums, the cmp result and an environment record.  Without it
# the temporary directory is deleted on exit and only the printed hashes remain.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(cd "${here}/.." && pwd)"
after_root="$(cd "${pkg}/../.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

evidence="${W0_EVIDENCE_DIR:-}"
if [[ -n "${evidence}" ]]; then
    mkdir -p "${evidence}"
    echo "[w0] evidence dir: ${evidence}"
fi

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
    if [[ -n "${evidence}" ]]; then
        cp "${work}/after.txt" "${evidence}/after.txt"
        sha256sum "${evidence}/after.txt" > "${evidence}/sha256sums.txt"
    fi
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

cmp_log="cmp -s before.txt after.txt"
if cmp -s "${work}/before.txt" "${work}/after.txt"; then
    echo "[w0] PASS: outputs are bit-identical"
    cmp_log="${cmp_log}  -> 0 (identical)"
else
    echo "[w0] FAIL: outputs differ"
    diff -u "${work}/before.txt" "${work}/after.txt" | head -20
    cmp_log="${cmp_log}  -> 1 (differ)"
fi

if [[ -n "${evidence}" ]]; then
    cp "${work}/before.txt" "${evidence}/before.txt"
    cp "${work}/after.txt"  "${evidence}/after.txt"
    ( cd "${evidence}" && sha256sum before.txt after.txt > sha256sums.txt )
    printf '%s\n' "${cmp_log}" > "${evidence}/cmp.log"
    {
        echo "generated_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "cwd: ${after_root}"
        echo "after_source: ${after_root} @ $(git -C "${after_root}" rev-parse HEAD 2>/dev/null || echo unknown)"
        echo "before_source: ${W0_BEFORE_ROOT} @ $(git -C "${W0_BEFORE_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
        echo "compiler: $(g++ --version | head -1)"
        echo "flags: ${flags[*]}"
        echo "ldd_libdual_arm_kinematics: $(sha256sum "${after_root}/install/lib/libdual_arm_kinematics.so" | cut -d' ' -f1)"
        echo "dump_source: ${here}/dump_anchored_task_samples.cpp  sha256=$(sha256sum "${here}/dump_anchored_task_samples.cpp" | cut -d' ' -f1)"
        echo "after_units: ${pkg}/src/anchored_object_task.cpp sha256=$(sha256sum "${pkg}/src/anchored_object_task.cpp" | cut -d' ' -f1)"
        echo "after_units: ${pkg}/src/trajectory.cpp sha256=$(sha256sum "${pkg}/src/trajectory.cpp" | cut -d' ' -f1)"
        echo "before_units: ${W0_BEFORE_ROOT}/src/dual_arm_transport_mock/src/object_task.cpp sha256=$(sha256sum "${W0_BEFORE_ROOT}/src/dual_arm_transport_mock/src/object_task.cpp" | cut -d' ' -f1)"
        echo "before_units: ${W0_BEFORE_ROOT}/src/dual_arm_transport_mock/src/trajectory.cpp sha256=$(sha256sum "${W0_BEFORE_ROOT}/src/dual_arm_transport_mock/src/trajectory.cpp" | cut -d' ' -f1)"
    } > "${evidence}/meta.txt"
    echo "[w0] evidence written to ${evidence}"
fi

grep -q '0 (identical)' <<<"${cmp_log}" || exit 1
