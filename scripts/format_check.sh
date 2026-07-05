#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

if [ ! -f "${VEXU_HOME}/.uncrustify.cfg" ]
then
    echo "Failure: .uncrustify.cfg not found at repo root."
    exit -1
fi

if ! command -v uncrustify >/dev/null 2>&1
then
    echo "Failure: uncrustify is not installed."
    exit -1
fi

cd "${VEXU_HOME}"

mapfile -t format_files < <(
    find "${VEXU_HOME}/01_Libraries" "${VEXU_HOME}/03_ROS" "${VEXU_HOME}/04_Sim" "${VEXU_HOME}/10_Examples" "${VEXU_HOME}/11_Robots" \
        \( -path '*/build/*' -o -path '*/install/*' \) -prune \
        -o \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' \) -print
)

if [ "${#format_files[@]}" -eq 0 ]
then
    echo "No C++ files found to check."
    exit 0
fi

echo "---Checking C++ formatting with uncrustify---"
uncrustify --check -c "${VEXU_HOME}/.uncrustify.cfg" "${format_files[@]}"
