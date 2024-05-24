# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Prerequisites
Ubuntu 22.04.

**For new team members doing Software Onboarding I, please start with** [Setting Up My Environment](https://github.com/VEXU-GHOST/VEXU_GHOST/blob/develop/SetupMyEnvironment.md).

## Installation
### Install ROS2 Humble

Follow Link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Repo Setup
#### Download Repository
```sh
cd
```
```
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
```
```
cd VEXU_GHOST
```
```
git submodule init
```
```
git submodule update --recursive
```
#### Add Setup to ~/.bashrc (which "configures" a new terminal when you open it)
```sh
echo "export VEXU_HOME=\"/home/$(whoami)/VEXU_GHOST\"" >> ~/.bashrc
```
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
```
echo "source ~/VEXU_GHOST/install/setup.bash" >> ~/.bashrc
```
```
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
```
```
echo 'export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
```
Close this terminal, and open a new one to load new settings.
#### Update Dependencies
```sh
./scripts/update_dependencies.sh
```

#### Build Submodules
```sh
source ~/.bashrc
```
```
./scripts/setup_submodules.sh
```

#### Build Repository
```sh
source ~/.bashrc
```
```
./scripts/build.sh -r
```

#### Start Simulator
```sh
source ~/.bashrc
```
```
./scripts/launch_sim.sh
```

#### Add yourself to the dialout group (only needed for real robot)

```sh
sudo usermod -a -G dialout $USER
```
