## Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots. The most useful module is likely the ghost_serial package, which allows users to directly address V5 Sensors and Actuators through ROS Topics on a Linux device.

#### Robust Features:
- Bidirectional Serial Communication module between a coprocessor and V5 Brain (address hardware directly from Ubuntu through ROS topics).
- LIDAR-based Particle Filter localization
- DC Motor Modeling / Simulation with Gazebo

#### Up-and-Coming Features:
- Model-Predictive Control for Differential Swerve Drive using CasADi and Ipopt
- Computer Vision pipeline for detection of competition game elements
- Behavior Tree based autonomy

## Prerequisites
Ubuntu 22.04.

If you are a regular Linux developer, continue as normal. For new team members, please see [Setting Up My Environment](https://github.com/VEXU-GHOST/VEXU_GHOST/blob/master/SetupMyEnvironment.md).

## Installation
#### Install ROS2 Humble

Follow Link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Repo Setup
#### Download Repository
```
cd
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
git submodule init
git submodule update --recursive
```
#### Add Setup to ~/.bashrc (which "configures" a new terminal when you open it)
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/VEXU_GHOST/install/setup.bash" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
```

#### Update Dependencies
```
./scripts/update_dependencies.sh
```

#### Build Submodules
```
./scripts/setup_submodules
```

### Build Repository
```
./scripts/build_ros.sh
```
