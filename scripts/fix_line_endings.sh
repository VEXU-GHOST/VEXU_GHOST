#!/usr/bin/env bash
# One-shot migration for checkouts made before .gitattributes pinned `eol=lf`.
#
# Symptom it fixes: scripts fail with `$'\r': command not found`, or
# `bad interpreter: No such file or directory`, because the worktree was checked
# out with CRLF (typically a Windows clone with core.autocrlf=true). That worktree
# is what docker-compose bind-mounts into the container, so the container sees the
# CRLF too.
#
# Run it once, from anywhere in the repo:
#
#     bash <(git show HEAD:scripts/fix_line_endings.sh)
#
# Use that form rather than `bash scripts/fix_line_endings.sh`: on a CRLF checkout
# *this file* is CRLF as well and bash dies on its own `set -euo pipefail` before
# doing anything. `git show` reads the blob from the object store, which is LF
# regardless of what the worktree looks like.
#
# Safe to re-run; it is a no-op once the worktree is clean.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || {
  echo "fix_line_endings: not inside a git repository." >&2
  exit 1
}
cd "$ROOT"

# `git ls-files --eol` reports, per tracked file: line endings in the index (i/),
# in the worktree (w/), and the effective attributes (attr/). Select files whose
# worktree copy is CRLF but whose attributes ask for LF.
#
# This is why we ask git instead of running `find | dos2unix` over an extension
# list. git already knows which files are text (binaries report `w/-text` and are
# never selected), which are tracked (no .git/, no build/, no install/), and which
# are *supposed* to be CRLF -- .bat/.cmd/.ps1 are `eol=crlf` in .gitattributes, and
# a blind CR-strip would corrupt exactly those.
mapfile -t offenders < <(
  git -c core.quotePath=false ls-files --eol \
    | awk -F'\t' '$1 ~ /w\/(crlf|mixed)/ && $1 ~ /eol=lf/ {print $2}'
)

if [ ${#offenders[@]} -eq 0 ]; then
  echo "fix_line_endings: worktree is already LF; nothing to do."
  exit 0
fi

printf 'fix_line_endings: converting %d file(s) to LF:\n' "${#offenders[@]}"
printf '  %s\n' "${offenders[@]}"

if command -v dos2unix >/dev/null 2>&1; then
  printf '%s\0' "${offenders[@]}" | xargs -0 dos2unix -q
else
  # sed is always present; dos2unix is not (and needs apt on a minimal image).
  printf '%s\0' "${offenders[@]}" | xargs -0 sed -i 's/\r$//'
fi

echo
echo "fix_line_endings: done. \`git status\` should still be clean -- these files"
echo "were already LF in the index, so this only realigns the worktree with it."
