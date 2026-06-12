#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

cd $VEXU_HOME

# If package names are given, clean only those packages' artifacts and exit.
if [ "$#" -gt 0 ];
then
    for pkg in "$@";
    do
        echo "Cleaning $pkg artifacts..."
        [ -d "build/$pkg" ] && sudo rm -r "build/$pkg"
        [ -d "install/$pkg" ] && sudo rm -r "install/$pkg"
    done
    exit 0
fi

[ -d "build" ] && sudo rm -r build/
[ -d "install" ] && sudo rm -r install/
[ -d "log" ] && sudo rm -r log/
