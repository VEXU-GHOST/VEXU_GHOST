
export ISAAC_ROS_WS=/home/ghost2/workspaces/isaac_ros-dev/

source /opt/ros/humble/setup.bash
source ~/VEXU_GHOST/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim:$GAZEBO_PLUGIN_PATH
