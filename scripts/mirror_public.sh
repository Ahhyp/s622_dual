#!/usr/bin/env bash
# Mirror an explicit list of files from the working branch (master) to the
# curated `public` branch, then fast-forward push it.
#
#   tools/mirror_public.sh <path> [<path> ...]
#
# `public` is not a mirror of master's history: it is a single-commit snapshot
# of the curated subtree (src/, docs/, README.md, CLAUDE.md, .gitignore).  That
# makes an accidental `git add -A` catastrophic -- it would publish whatever
# happened to be dirty in the working tree.  This script exists because that is
# exactly what happened once during W2: a failed `git worktree add` did not stop
# the caller, and `git add -A` turned the user's uncommitted notes into a commit.
#
# Hard rules enforced here:
#   1. no `git add -A`: only the paths named on the command line are staged;
#   2. every path must be inside the curated subtree (ALLOW below);
#   3. abort if the mirror worktree cannot be created, or cannot be entered;
#   4. abort before committing if the staged list differs from the request;
#   5. abort before pushing unless the remote is an ancestor (fast-forward);
#   6. never force push, never reset --hard, never clean -fd;
#   7. nothing is committed until every check above has passed.
#
# Env overrides (used by tools/mirror_public_selftest.sh):
#   MIRROR_WORKTREE  worktree path for `public`   (default /tmp/pub_mirror)
#   MIRROR_REMOTE    remote name                  (default origin)
#   MIRROR_FROM      branch to mirror from        (default master)
#   MIRROR_MSG       commit message               (default derived)
#   MIRROR_DRY_RUN   1 => stop after the staged-list check

set -euo pipefail

die() {
    printf 'ABORT: %s\n' "$*" >&2
    exit 1
}

ALLOW=("src/" "docs/" "README.md" "CLAUDE.md" ".gitignore")

WORKTREE="${MIRROR_WORKTREE:-/tmp/pub_mirror}"
REMOTE="${MIRROR_REMOTE:-origin}"
FROM="${MIRROR_FROM:-master}"
DRY_RUN="${MIRROR_DRY_RUN:-0}"

(( $# > 0 )) || die "no paths given: this script never stages on its own"

# ---- 8. preconditions on the caller's tree ---------------------------------
ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || die "not inside a git work tree"
[[ "$(pwd -P)" == "$(cd "$ROOT" && pwd -P)" ]] || die "must be run from the repository root ($ROOT)"
[[ -d "$ROOT/.git" ]] || die "run from the main worktree, not from a linked one"
BRANCH="$(git rev-parse --abbrev-ref HEAD)"
[[ "$BRANCH" == "$FROM" ]] || die "expected to be on $FROM, currently on $BRANCH"
git rev-parse --verify --quiet "$FROM" >/dev/null || die "no branch $FROM"
git rev-parse --verify --quiet "public" >/dev/null || die "no local branch 'public'"

# ---- validate the request against the allow-list ---------------------------
declare -a WANTED=()
for p in "$@"; do
    [[ "$p" == /* ]] && die "absolute path not allowed: $p"
    [[ "$p" == *".."* ]] && die "path traversal not allowed: $p"
    ok=0
    for a in "${ALLOW[@]}"; do
        if [[ "$p" == "$a"* ]]; then ok=1; break; fi
    done
    (( ok == 1 )) || die "path outside the curated subtree: $p"
    git cat-file -e "$FROM:$p" 2>/dev/null || die "path is not tracked on $FROM: $p"
    WANTED+=("$p")
done

# ---- 3. create and enter the mirror worktree ------------------------------
[[ -e "$WORKTREE" ]] && die "mirror worktree target already exists: $WORKTREE (remove it first)"
cleanup() {
    git worktree remove --force "$WORKTREE" >/dev/null 2>&1 || true
}
trap cleanup EXIT
git worktree add --quiet "$WORKTREE" public || die "git worktree add failed for $WORKTREE"
cd "$WORKTREE" || die "cannot enter mirror worktree $WORKTREE"
[[ "$(git rev-parse --abbrev-ref HEAD)" == "public" ]] ||
    die "mirror worktree is not on 'public' but on '$(git rev-parse --abbrev-ref HEAD)'"

# ---- 1./2. copy exactly the requested files -------------------------------
for p in "${WANTED[@]}"; do
    mkdir -p "$(dirname "$p")"
    git --git-dir="$ROOT/.git" show "$FROM:$p" > "$p" || die "cannot read $FROM:$p"
done
git add -- "${WANTED[@]}"

# ---- 4. the staged list must equal the request ----------------------------
# -z + core.quotePath=false: git quotes non-ASCII paths by default, so a plain
# `--name-only` returns "docs/\345\217\214..." for CJK filenames and every
# comparison below would fail.  (Found by tools/mirror_public_selftest.sh.)
mapfile -d '' -t STAGED < <(git -c core.quotePath=false diff --cached --name-only -z)
printf 'staged (%d):\n' "${#STAGED[@]}"
printf '  %s\n' "${STAGED[@]}"
[[ "${#STAGED[@]}" -gt 0 ]] || die "nothing staged: the requested files are already identical"
for s in "${STAGED[@]}"; do
    found=0
    for w in "${WANTED[@]}"; do [[ "$s" == "$w" ]] && found=1 && break; done
    (( found == 1 )) || die "staged a path that was not requested: $s"
done
printf 'diff vs previous public tip:\n'
git -c core.quotePath=false diff --cached --name-status

if [[ "$DRY_RUN" == "1" ]]; then
    echo "DRY RUN: stopping before commit"
    exit 0
fi

# ---- commit ---------------------------------------------------------------
MSG="${MIRROR_MSG:-mirror: $(git -C "$ROOT" log -1 --pretty=%s "$FROM")}"
git commit --quiet -m "$MSG" || die "commit failed"
echo "committed: $(git log -1 --pretty='%h %s')"

# ---- 5. fast-forward only push -------------------------------------------
git fetch --quiet "$REMOTE" public || die "git fetch $REMOTE public failed"
REMOTE_TIP="$(git rev-parse "refs/remotes/$REMOTE/public" 2>/dev/null || echo '')"
[[ -n "$REMOTE_TIP" ]] || die "cannot resolve $REMOTE/public after fetch"
if ! git merge-base --is-ancestor "$REMOTE_TIP" HEAD; then
    die "remote $REMOTE/public ($REMOTE_TIP) is not an ancestor of HEAD: refusing to push (no force)"
fi
# Proxy: the machine's global git config points at an HTTP proxy that cannot
# complete a TLS handshake; clearing it per-invocation is the working setup.
git -c http.proxy= -c https.proxy= push "$REMOTE" public || die "push failed"
echo "pushed: $(git rev-parse --short HEAD) -> $REMOTE/public"
