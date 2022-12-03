# Setting Up ROS container

Build the Dockerfile by naviagting to the directory with the Dockerfile and using the command `docker build . -t ros2-foxy`.

Create a Docker ROS container using `docker run -it --name LIDAR ros2-foxy`. You're now running bash in the container. Type `exit` to stop.
To re-enter the Docker container, run `docker container start LIDAR`. Confirm it's running with `docker ps` and then run `docker exec -it LIDAR bash`. 
You may need to run `. /opt/ros/foxy/setup.bash` to get the ros2 command working.

Setup X11 forwarding. This may take a while.
For MAC, I found these links useful
[https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088)
[https://medium.com/@mreichelt/how-to-show-x11-windows-within-docker-on-mac-50759f4b65cb](https://medium.com/@mreichelt/how-to-show-x11-windows-within-docker-on-mac-50759f4b65cb)
Get to the point where running `xeyes` in the ROS docker container openes an xeyes windows on your computer.

Download this repo (from this branch) and copy it into the Docker container using the `docker cp VEXU_GHOST LIDAR:root/VEXU_GHOST`. Then re-enter the container and
navigate to the scripts folder of VEXU_GHOST (note that VEXU_GHOST is in the root directory and you will have to enter `cd ~` to navigate to this directory). Run the following commands _in order_ from the VEXU_GHOST directory:
`bash install_dependencies.sh`
`bash build.sh`
`bash launch_sim.sh`

error possibly in ~/VEXU_GHOST/install/setup.bash
    COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"

~/VEXU_GHOST/install/ghost_sim/share/ghost_sim/package.bash


source /opt/ros/foxy/setup.bash
source /root/VEXU_GHOST/install/setup.bash
export DISPLAY=host.docker.internal:0
export LIBGL_ALWAYS_INDIRECT=1

https://twiki.nevis.columbia.edu/twiki/bin/view/Main/X11OnLaptops
http://tleyden.github.io/
