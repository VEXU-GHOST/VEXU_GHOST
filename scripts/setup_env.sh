: "${VEXU_HOME:=~/VEXU_GHOST}"

export ISAAC_ROS_WS=/home/ghost/workspaces/isaac_ros-dev/

source /opt/ros/humble/setup.bash
source $VEXU_HOME/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$VEXU_HOME/build/ghost_sim:$GAZEBO_PLUGIN_PATH
