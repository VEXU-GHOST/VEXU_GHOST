#!/usr/bin/env bash
# One-shot fix for a CRLF checkout made before .gitattributes pinned eol=lf.
# Symptoms: "$'\r': command not found", "/bin/bash^M: bad interpreter".
#
#   bash <(git show HEAD:scripts/fix_line_endings.sh)
#
# Use that form, not `bash scripts/fix_line_endings.sh` -- on a CRLF checkout this
# file is CRLF too and bash dies on the `set` line below. The blob git show reads
# is LF regardless. Safe to re-run; no-op on a healthy checkout.
set -euo pipefail

cd "$(git rev-parse --show-toplevel 2>/dev/null)" || {
  echo "fix_line_endings: not inside a git repository." >&2; exit 1
}

# Tracked files that are CRLF in the worktree but LF by attribute. Asking git rather
# than globbing gets us four exclusions for free: .git/, build/ + install/, submodule
# contents, and binaries (meshes, images -- git reports those w/-text, and a blind
# CR-strip would corrupt them). It also honors the eol=crlf .bat/.cmd/.ps1 rule.
mapfile -t files < <(
  git ls-files --eol | awk -F'\t' '$1 ~ /w\/(crlf|mixed)/ && $1 ~ /eol=lf/ {print $2}'
)

if [ ${#files[@]} -eq 0 ]; then
  echo "fix_line_endings: already LF, nothing to do."
  exit 0
fi

printf 'fix_line_endings: converting %d file(s):\n' "${#files[@]}"
printf '  %s\n' "${files[@]}"

# Prefer dos2unix: it does its own binary sniffing, so a mistake in the selection
# above can't corrupt a mesh. Install it if apt is around (Linux host, or inside the
# container -- the image doesn't ship it). macOS and Git Bash have no apt, so those
# fall through to sed, which is fine given the selection is already binary-safe.
if ! command -v dos2unix >/dev/null 2>&1 && command -v apt-get >/dev/null 2>&1; then
  SUDO=""
  [ "$(id -u)" -ne 0 ] && SUDO="sudo"
  echo "fix_line_endings: installing dos2unix..."
  $SUDO apt-get update -qq \
    && $SUDO apt-get install -y -qq --no-install-recommends dos2unix \
    || echo "fix_line_endings: dos2unix install failed, falling back to sed." >&2
fi

if command -v dos2unix >/dev/null 2>&1; then
  printf '%s\0' "${files[@]}" | xargs -0 dos2unix -q
else
  printf '%s\0' "${files[@]}" | xargs -0 sed -i 's/\r$//'
fi

echo "fix_line_endings: done -- no commit needed, these were already LF in the index."
