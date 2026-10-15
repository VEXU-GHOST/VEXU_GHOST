#!/bin/bash
# Resolves a segfault address to a source location.
# Usage: pros_debug_stacktrace.sh <address> [project_dir]

: "${VEXU_HOME:=$HOME/VEXU_GHOST}"
PROJECT_DIR="${2:-02_V5/ghost_pros}"

if [ -z $1 ]
then
echo "Error: Script requires segfault address."
exit
fi

arm-none-eabi-addr2line -faps -e $VEXU_HOME/$PROJECT_DIR/bin/hot.package.elf $1
