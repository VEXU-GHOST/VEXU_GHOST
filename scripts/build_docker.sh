#!/bin/bash -e

SCRIPT_DIR=$(dirname $(realpath $0))
REPO_DIR=$(dirname $SCRIPT_DIR)

cd $REPO_DIR/docker
docker build --tag vexu_ghost .