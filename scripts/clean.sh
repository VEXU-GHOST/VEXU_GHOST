#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

cd $VEXU_HOME

[ -d "build" ] && sudo rm -r build/
[ -d "install" ] && sudo rm -r install/
[ -d "log" ] && sudo rm -r log/
