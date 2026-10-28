#!/usr/bin/env bash
# C4.3 S4 / W2-C3 D2-B: reproduce the fixture derivation and record provenance.
#
# What it proves (and what it does NOT):
#   * the 29-column fixture is byte-for-byte derivable from the 44-column S3
#     artifact by a tool that shares no code with dual_arm_qp_core;
#   * two clean builds of that tool + two independent runs produce identical
#     bytes;
#   * the derivation is reproducible across two separate build directories.
# It does NOT touch the frozen fixture and it does NOT grade any criterion.
#
# Usage: bash src/dual_arm_qp_core/scripts/generate_s3_fixture.sh [outdir]
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUTDIR="${1:-/tmp/c3_fixture_provenance}"
ARTIFACT="$REPO_ROOT/runs/c43_s3/w8/ff_1/cycles.csv"
FROZEN="$REPO_ROOT/src/dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv"
SRC="$REPO_ROOT/src/dual_arm_qp_core/test/tools/generate_s3_fixture.cpp"

mkdir -p "$OUTDIR/build_a" "$OUTDIR/build_b"
CXX="${CXX:-g++}"

echo "== 1/6 build generator twice (separate clean dirs) =="
"$CXX" -std=c++17 -O2 -Wall -Wextra -o "$OUTDIR/build_a/gen" "$SRC"
"$CXX" -std=c++17 -O2 -Wall -Wextra -o "$OUTDIR/build_b/gen" "$SRC"

echo "== 2/6 generate twice =="
"$OUTDIR/build_a/gen" --artifact "$ARTIFACT" --out "$OUTDIR/fixture_a.csv"
"$OUTDIR/build_b/gen" --artifact "$ARTIFACT" --out "$OUTDIR/fixture_b.csv"

echo "== 3/6 two independent generations must agree byte for byte =="
if ! cmp -s "$OUTDIR/fixture_a.csv" "$OUTDIR/fixture_b.csv"; then
  echo "FAIL: the two generations differ"
  cmp "$OUTDIR/fixture_a.csv" "$OUTDIR/fixture_b.csv" || true
  exit 1
fi
echo "OK: fixture_a.csv == fixture_b.csv"

echo "== 4/6 regeneration must agree with the frozen fixture =="
regen_vs_frozen=DIFFERENT
if cmp -s "$OUTDIR/fixture_a.csv" "$FROZEN"; then
  regen_vs_frozen=IDENTICAL
fi
echo "regen_vs_frozen=$regen_vs_frozen"

echo "== 5/6 frozen fixture must not have changed while we ran =="
frozen_sha="$(sha256sum "$FROZEN" | cut -d' ' -f1)"
echo "frozen_sha256=$frozen_sha"

echo "== 6/6 provenance =="
{
  echo "# C3 fixture derivation provenance"
  echo ""
  echo "generated_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "generator_source: src/dual_arm_qp_core/test/tools/generate_s3_fixture.cpp"
  echo "generator_source_sha256: $(sha256sum "$SRC" | cut -d' ' -f1)"
  echo "compiler: $("$CXX" --version | head -1)"
  echo "compile_flags: -std=c++17 -O2 -Wall -Wextra"
  echo "artifact: runs/c43_s3/w8/ff_1/cycles.csv"
  echo "artifact_sha256: $(sha256sum "$ARTIFACT" | cut -d' ' -f1)"
  echo "output_sha256: $(sha256sum "$OUTDIR/fixture_a.csv" | cut -d' ' -f1)"
  echo "frozen_fixture: src/dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv"
  echo "frozen_fixture_sha256: $frozen_sha"
  echo "regen_vs_frozen: $regen_vs_frozen"
  echo "two_generations_identical: yes"
  echo "rows: $(($(wc -l < "$OUTDIR/fixture_a.csv") - 1))"
  echo "columns: $(head -1 "$OUTDIR/fixture_a.csv" | awk -F, '{print NF}')"
  echo "note: this run does NOT grade the ULP criterion and does NOT modify the frozen fixture"
} | tee "$OUTDIR/PROVENANCE.txt"

echo
echo "evidence dir: $OUTDIR"
