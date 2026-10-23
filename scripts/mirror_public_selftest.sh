#!/usr/bin/env bash
# Fault injection for tools/mirror_public.sh (C4.3 S4 W2 addendum, task D2).
#
# The W2 incident was not "git is dangerous": it was that a failed
# `git worktree add` did not stop the caller, and an unconstrained `git add -A`
# then published the user's uncommitted notes.  A guard is only worth having if
# each of those failure paths is shown to STOP.  This script builds a throwaway
# clone, injects four faults, and prints the exit code of every case.
#
#   C1  mirror worktree target already exists      -> abort before touching git
#   C2  invoked from a subdirectory                -> abort (root/branch check)
#   C3  an unrequested path ends up staged         -> abort before commit
#   C4  remote public tip is not an ancestor       -> abort before push
#   P1  positive control: clean dry run            -> exit 0
#
# Run: tools/mirror_public_selftest.sh [--keep]

set -uo pipefail

REPO_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
KEEP=0
[[ "${1:-}" == "--keep" ]] && KEEP=1
ROOT="$(mktemp -d /tmp/mirror_selftest.XXXXXX)"
FAILED=0

say() { printf '\n=== %s ===\n' "$*"; }
verdict() {  # name expected_rc actual_rc
    if [[ "$2" == "nonzero" ]]; then
        if [[ "$3" != "0" ]]; then printf 'PASS: %s aborted (exit %s)\n' "$1" "$3"
        else printf 'FAIL: %s did NOT abort (exit 0)\n' "$1"; FAILED=1; fi
    elif [[ "$3" == "0" ]]; then printf 'PASS: %s (exit 0)\n' "$1"
    else printf 'FAIL: %s failed with exit %s\n' "$1" "$3"; FAILED=1; fi
}

cleanup() {
    if [[ "$KEEP" == "1" ]]; then
        printf '\nkept scratch tree: %s\n' "$ROOT"
    else
        rm -rf "$ROOT"
    fi
}
trap cleanup EXIT

# ---- throwaway clone (never touches the real repository) -------------------
say "setup"
git clone --quiet --no-hardlinks "$REPO_SRC" "$ROOT/repo" || { echo "clone failed"; exit 1; }
REPO="$ROOT/repo"
git -C "$REPO" checkout --quiet master
git -C "$REPO" branch --quiet public origin/public
git -C "$REPO" config user.email selftest@example.com
git -C "$REPO" config user.name selftest
git -C "$REPO" config http.proxy ""
git -C "$REPO" config https.proxy ""
printf 'scratch repo: %s (master=%s public=%s)\n' "$REPO" \
    "$(git -C "$REPO" rev-parse --short master)" "$(git -C "$REPO" rev-parse --short public)"

# A file that exists on master and inside the curated subtree, used as the
# nominal mirror request.  It is modified in the scratch clone (never in the real
# tree) so that there is always something to stage -- otherwise the run aborts
# with "nothing staged" before reaching the checks under test.
TARGET="docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_W2_验收记录.md"
[[ -e "$REPO/$TARGET" ]] || { echo "missing test fixture $TARGET"; exit 1; }
printf '\n<!-- selftest edit -->\n' >> "$REPO/$TARGET"
git -C "$REPO" add -- "$TARGET"
git -C "$REPO" commit --quiet -m "selftest: modify the mirrored file"
# The guard itself may be newer than the last commit; copy the working copy of
# tools/ in so the selftest always exercises what is on disk.
cp -a "$REPO_SRC/tools/." "$REPO/tools/" 2>/dev/null || mkdir -p "$REPO/tools"
SCRIPT="$REPO/tools/mirror_public.sh"
[[ -f "$SCRIPT" ]] || { echo "missing $SCRIPT"; exit 1; }
chmod +x "$SCRIPT"

# ---- P1: clean dry run ----------------------------------------------------
say "P1 positive control (clean dry run)"
( cd "$REPO" && MIRROR_WORKTREE="$ROOT/wt_p1" MIRROR_DRY_RUN=1 \
    "$SCRIPT" "$TARGET" ) > "$ROOT/p1.log" 2>&1
rc=$?
verdict "P1 clean dry run" "zero" "$rc"
tail -5 "$ROOT/p1.log" | sed 's/^/    /'

# ---- C1: worktree target already exists -----------------------------------
say "C1 mirror worktree occupied"
mkdir -p "$ROOT/wt_c1"
( cd "$REPO" && MIRROR_WORKTREE="$ROOT/wt_c1" MIRROR_DRY_RUN=1 \
    "$SCRIPT" "$TARGET" ) > "$ROOT/c1.log" 2>&1
rc=$?
verdict "C1 occupied worktree" "nonzero" "$rc"
grep -m1 'ABORT' "$ROOT/c1.log" | sed 's/^/    /'

# ---- C2: invoked from a subdirectory --------------------------------------
say "C2 invoked from a subdirectory"
mkdir -p "$REPO/src/sub"
( cd "$REPO/src/sub" && MIRROR_WORKTREE="$ROOT/wt_c2" MIRROR_DRY_RUN=1 \
    "$SCRIPT" "$TARGET" ) > "$ROOT/c2.log" 2>&1
rc=$?
verdict "C2 wrong working directory" "nonzero" "$rc"
grep -m1 'ABORT' "$ROOT/c2.log" | sed 's/^/    /'

# ---- C3: an unrequested path ends up staged -------------------------------
# Injects the incident itself: after the script's own `git add`, a stray file is
# staged behind its back.  The staged-vs-requested check must catch it.
say "C3 unrequested path staged (fault injection)"
mkdir -p "$ROOT/bin"
cat > "$ROOT/bin/git" <<'SHIM'
#!/usr/bin/env bash
/usr/bin/git "$@"
rc=$?
if [[ "${1:-}" == "add" ]]; then
    printf 'stray\n' > docs/SELFTEST_STRAY.md
    /usr/bin/git add -- docs/SELFTEST_STRAY.md >/dev/null 2>&1
fi
exit "$rc"
SHIM
chmod +x "$ROOT/bin/git"
( cd "$REPO" && PATH="$ROOT/bin:$PATH" MIRROR_WORKTREE="$ROOT/wt_c3" MIRROR_DRY_RUN=1 \
    "$SCRIPT" "$TARGET" ) > "$ROOT/c3.log" 2>&1
rc=$?
verdict "C3 unrequested staged path" "nonzero" "$rc"
grep -m1 'ABORT' "$ROOT/c3.log" | sed 's/^/    /'

# ---- C4: remote public tip is not an ancestor -----------------------------
say "C4 remote public diverged (no force)"
git init --quiet --bare "$ROOT/fake.git"
git -C "$REPO" push --quiet "$ROOT/fake.git" public:refs/heads/public
git -C "$REPO" checkout --quiet -B selftest_extra public
printf 'x\n' > "$REPO/SELFTEST_EXTRA.txt"
git -C "$REPO" add SELFTEST_EXTRA.txt
git -C "$REPO" commit --quiet -m "divergent public commit"
git -C "$REPO" push --quiet "$ROOT/fake.git" selftest_extra:refs/heads/public
git -C "$REPO" checkout --quiet master
git -C "$REPO" branch --quiet -D selftest_extra
( cd "$REPO" && MIRROR_REMOTE="$ROOT/fake.git" MIRROR_WORKTREE="$ROOT/wt_c4" \
    "$SCRIPT" "$TARGET" ) > "$ROOT/c4.log" 2>&1
rc=$?
verdict "C4 diverged remote" "nonzero" "$rc"
grep -m1 'ABORT' "$ROOT/c4.log" | sed 's/^/    /'

# ---- final state of the caller's real tree --------------------------------
say "real tree untouched"
git -C "$REPO_SRC" rev-parse --abbrev-ref HEAD | sed 's/^/    branch: /'
git -C "$REPO_SRC" rev-parse --short HEAD | sed 's/^/    HEAD:   /'
printf '    worktrees of the real repo: %s\n' \
    "$(git -C "$REPO_SRC" worktree list | wc -l)"

if [[ "$FAILED" == "0" ]]; then
    echo "RESULT: OK (4/4 faults stopped, positive control passed)"
else
    echo "RESULT: FAILED"
fi
exit "$FAILED"
