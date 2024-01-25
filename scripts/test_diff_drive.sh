#!/bin/bash
cd ~/VEXU_GHOST
cd 03_ROS/ghost_sim/urdf

# Make description sdf file from xacro file
ros2 run xacro xacro --inorder test_tank_init.xacro > test_tank_init.urdf
gz sdf -p test_tank_init.urdf > test_tank_init.sdf

echo "Allow generated test description files to be moved to temp testing folder"
# sudo cp test_tank_init.urdf ~/VEXU_GHOST/03_ROS/ghost_sim/Testing/Temporary
# sudo cp test_tank_init.sdf ~/VEXU_GHOST/03_ROS/ghost_sim/Testing/Temporary

# Make sure gazebo is dead, just in case
echo "Killing gazebo processes"
killall gzserver
sleep 2

cd ~/VEXU_GHOST
cd build/ghost_sim
ctest test_diff_drive --verbose
rm -f test_tank_init.urdf
rm -f test_tank_init.sdf
