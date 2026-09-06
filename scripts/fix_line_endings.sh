#!/usr/bin/env bash
# One-shot fix for a CRLF checkout made before .gitattributes pinned eol=lf.
# Symptoms: "$'\r': command not found", "/bin/bash^M: bad interpreter".
#
#   bash <(git show HEAD:scripts/fix_line_endings.sh)
#
# Use that form, not `bash scripts/fix_line_endings.sh` -- on a CRLF checkout this
# file is CRLF too and bash dies on the `set` line below. Safe to re-run.
set -eu

cd "$(git rev-parse --show-toplevel)"

if ! command -v dos2unix >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    SUDO=$([ "$(id -u)" = 0 ] || echo sudo)
    $SUDO apt-get update -qq
    $SUDO apt-get install -y -qq --no-install-recommends dos2unix
  else
    echo "fix_line_endings: need dos2unix (brew install dos2unix)." >&2
    exit 1
  fi
fi

# git ls-files, not a glob: `**/*` isn't recursive without `shopt -s globstar`, and
# with it you sweep in build/, install/ and submodules. dos2unix skips binaries and
# files that are already LF on its own, so no filtering is needed here.
git ls-files -z | xargs -0 dos2unix -q

echo "fix_line_endings: done -- no commit needed, these were already LF in the index."
