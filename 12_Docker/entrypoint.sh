#!/usr/bin/env bash
# Do not use nounset: ROS setup scripts reference optional env vars.
set -eo pipefail

export VEXU_HOME="${VEXU_HOME:-/vexu}"
export VEXU_IN_DOCKER=1

# Cross-platform Docker / macOS: avoid multicast DDS issues in a single container.
if [[ -z "${ROS_LOCALHOST_ONLY:-}" ]]; then
  export ROS_LOCALHOST_ONLY=1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

if [[ -n "${VEXU_IN_DOCKER:-}" ]]; then
  if [[ -f "$VEXU_HOME/install-docker/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$VEXU_HOME/install-docker/setup.bash"
  fi
else
  if [[ -f "$VEXU_HOME/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$VEXU_HOME/install/setup.bash"
  fi
fi

export LD_LIBRARY_PATH="/usr/local/lib/:${LD_LIBRARY_PATH:-}"
COLCON_BUILD_ROOT="${VEXU_COLCON_BUILD_BASE:-build}"
if [[ -d "$VEXU_HOME/${COLCON_BUILD_ROOT}/ghost_sim" ]]; then
  export GAZEBO_PLUGIN_PATH="$VEXU_HOME/${COLCON_BUILD_ROOT}/ghost_sim:${GAZEBO_PLUGIN_PATH:-}"
fi

exec "$@"
