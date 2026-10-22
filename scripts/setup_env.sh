: "${VEXU_HOME:=$HOME/VEXU_GHOST}"
. "$VEXU_HOME/scripts/robot_name.sh"

export ISAAC_ROS_WS=/home/ghost/workspaces/isaac_ros-dev/

source /opt/ros/humble/setup.bash
source $VEXU_HOME/install/setup.bash

# Confine Fast DDS to the wired ROS network, but only on the robot (where the
# whitelisted IP exists). Remotes sourcing this file have no ~/robot_name,
# so they skip it and use default DDS discovery. See config/dds/eth_only.xml.
if [ -n "$ROBOT_NAME" ] && [ -f "$VEXU_HOME/config/dds/eth_only.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$VEXU_HOME/config/dds/eth_only.xml"
fi

export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$VEXU_HOME/build/ghost_sim:$GAZEBO_PLUGIN_PATH
export PATH="/usr/lib/ccache/:$PATH"
