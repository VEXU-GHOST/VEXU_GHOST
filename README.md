# VEXU_GHOST

## Requirements
Ubuntu 22.04

### SSH Setup 
If you just setup Github, you will need to add SSH keys. These let github recognize your computer (and this replaced using password last year).

Generate Key: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
Add Key: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

### Install ROS2

Follow Link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Repo Setup

Download Repository and Submodules
```
cd ~
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
git submodule init
git submodule update --recursive
```
### Update ~/.bashrc
This script is executed whenever you open a new terminal. The following lines will make your terminal "aware" of ROS libraries and of our codebase.
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/VEXU_GHOST/install/setup.bash" >> ~/.bashrc
```
### Update ROS Dependencies
```
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

### Build Submodules
```
./scripts/setup_submodules
```

### Build Repository
```
./scripts/build_ros.sh
```
