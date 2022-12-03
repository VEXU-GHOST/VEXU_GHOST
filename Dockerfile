FROM ros:foxy

# install ros package
RUN sudo apt-get -y update && \
    apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-foxy-rviz2 \
      nano \
      x11-apps && \
    rm -rf /var/lib/apt/lists/* && \
    sudo apt update && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

COPY . /root/VEXU_GHOST

RUN mkdir /root/VEXU_GHOST/ghost_sim/rviz && \
    bash /root/VEXU_GHOST/scripts/install_dependencies.sh && \
