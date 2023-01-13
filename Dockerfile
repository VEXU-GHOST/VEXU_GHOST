FROM ros:foxy

ARG USERNAME=lidar

RUN apt-get clean

# install ros package
RUN sudo apt-get update -y && \
    apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-foxy-rviz2 \
      nano \
      x11vnc \ 
      xvfb \
      xfce4 lightdm \
      xfce4-goodies \
      openssh-server && \
    rm -rf /var/lib/apt/lists/* && \
    sudo apt update -y

RUN echo "sudo service ssh start" >> ~/.bashrc \
    echo "export ROS_DISTRO=foxy" >> ~/.bashrc

RUN useradd -ms /bin/bash lidar sudo

USER lidar


# xauth add thing https://unix.stackexchange.com/questions/476290/x11-forwarding-does-not-work-if-su-to-another-user 

COPY . /root/VEXU_GHOST

RUN mkdir /root/VEXU_GHOST/ghost_sim/rviz && \
    /root/VEXU_GHOST/scripts/install_dependencies.sh


# useradd -ms /bin/bash test