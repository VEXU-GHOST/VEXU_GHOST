#!/usr/bin/env bash
# Sources ROS underlay + Ghost overlay, then exec's the user's command.
# All env (VEXU_HOME, VEXU_IN_DOCKER, ROS_LOCALHOST_ONLY, LD_LIBRARY_PATH,
# GAZEBO_PLUGIN_PATH) is set in the Dockerfile / docker-compose.yml.
# No `set -u`: ROS setup scripts read optional env vars.
set -eo pipefail

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

INSTALL_BASE="${VEXU_COLCON_INSTALL_BASE:-install}"
if [[ -f "$VEXU_HOME/$INSTALL_BASE/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$VEXU_HOME/$INSTALL_BASE/setup.bash"
fi

exec "$@"
