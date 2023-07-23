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
Download Repository
```
cd ~
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
```

Add Setup to ~/.bashrc (which "configures" a new terminal when you open it)
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/VEXU_GHOST/install/setup.bash" >> ~/.bashrc

echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
```

Update Dependencies
```
./scripts/update_dependencies.sh
```

Build Submodules
```
./scripts/build_submodules
```

