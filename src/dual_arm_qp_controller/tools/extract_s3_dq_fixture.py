#!/usr/bin/env python3
"""Regenerate test/fixtures/s3_ff_1_dq.csv from an S3 mock-loop run.

C4.3 S4.1 W2 (S4-T2) needs the S3 `ff_1` cycle sequence as a frozen baseline.
`runs/` is gitignored, so the sequence is extracted once into a versioned
fixture.  This script is the reproducible way to redo that extraction -- it does
not compute anything, it only copies columns and records provenance.

Usage:
    python3 tools/extract_s3_dq_fixture.py [--run runs/c43_s3/w8/ff_1] [--out test/fixtures]
"""
import argparse
import csv
import hashlib
import pathlib
import sys
import textwrap

COLS_INT = 12


def main() -> int:
    here = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="runs/c43_s3/w8/ff_1",
                    help="S3 run directory (relative to the repo root)")
    ap.add_argument("--out", default=str(here / "test" / "fixtures"))
    ap.add_argument("--repo", default=str(here.parent.parent))
    args = ap.parse_args()

    src = pathlib.Path(args.repo) / args.run / "cycles.csv"
    if not src.is_file():
        print(f"ERROR: {src} not found; run the S3 mock loop first", file=sys.stderr)
        return 2

    rows = list(csv.DictReader(src.open()))
    header = (["k", "t"]
              + [f"q_prev{i}" for i in range(COLS_INT)]
              + [f"dq{i}" for i in range(COLS_INT)]
              + ["e_obj_pos_m", "e_obj_rot_rad", "committed"])
    out_lines = [",".join(header)]
    for r in rows:
        q = r["q_prev"].split()
        d = r["dq"].split()
        if len(q) != COLS_INT or len(d) != COLS_INT:
            print(f"ERROR: row k={r['k']} has {len(q)} q_prev / {len(d)} dq", file=sys.stderr)
            return 3
        out_lines.append(",".join([r["k"], r["t"]] + q + d
                                  + [r["e_obj_pos_m"], r["e_obj_rot_rad"], r["committed"]]))

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    fixture = out_dir / "s3_ff_1_dq.csv"
    fixture.write_text("\n".join(out_lines) + "\n")
    sha = hashlib.sha256(src.read_bytes()).hexdigest()
    fsha = hashlib.sha256(fixture.read_bytes()).hexdigest()
    (out_dir / "PROVENANCE.txt").write_text(textwrap.dedent(f"""\
        # Frozen baseline for C4.3 S4.1 W2 (S4-T2): the S3 `ff_1` cycle sequence.
        #
        # source run dir : {args.run}/
        # source file    : cycles.csv
        # source sha256  : {sha}
        # rows           : {len(rows)}
        # fixture sha256 : {fsha}
        #
        # Regenerate with: python3 tools/extract_s3_dq_fixture.py --run {args.run}
        # Nothing is recomputed: every number is copied verbatim from cycles.csv.
        """))
    print(f"wrote {fixture} ({len(rows)} rows)")
    print(f"source sha256  {sha}")
    print(f"fixture sha256 {fsha}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
