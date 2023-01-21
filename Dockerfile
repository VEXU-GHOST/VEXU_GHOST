FROM ros:foxy

ARG USERNAME=lidar
ARG PASSWORD=password

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get clean

# install ros package
RUN sudo apt-get update -y && \
    apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-foxy-rviz2 \
      openssh-server \
      lightdm \
      ubuntu-desktop \
      nano \
      x11-apps \
      x11vnc \ 
      xvfb \
      xorg \
      xfce4 \
      xfce4-goodies && \
    rm -rf /var/lib/apt/lists/* && \
    sudo apt update -y

COPY . /root/VEXU_GHOST

RUN mkdir /root/VEXU_GHOST/ghost_sim/rviz && \
    /root/VEXU_GHOST/scripts/install_dependencies.sh

RUN echo "sudo service ssh start" >> ~/.bashrc && \
    echo "export ROS_DISTRO=foxy" >> ~/.bashrc && \
    echo "source /ros_entrypoint.sh" >> ~/.bashrc

RUN sudo useradd -m -p $(openssl passwd -1 "$PASSWORD") $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod -s /bin/bash $USERNAME

# RUN --user=$USERNAME xauth list $DISPLAY | xauth add -

# USER lidar

# RUN export XAUTH="$(xauth list $DISPLAY)"

# USER root

# RUN xauth add ${XAUTH}

# xauth add thing https://unix.stackexchange.com/questions/476290/x11-forwarding-does-not-work-if-su-to-another-user 