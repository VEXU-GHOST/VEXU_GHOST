FROM ros:foxy

ARG USERNAME=lidar
ARG PASSWORD=password

ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-c"]

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
      xterm \
      x11-apps \
      x11vnc \ 
      xvfb \
      xorg \
      xfce4 \
      xfce4-goodies && \
    rm -rf /var/lib/apt/lists/* && \
    sudo apt update -y

RUN touch ~/.Xauthority

RUN echo "sudo service ssh start" >> ~/.bashrc && \
    echo "export TERM=xterm" >> ~/.bashrc && \
    echo "export ROS_DISTRO=foxy" >> ~/.bashrc && \
    echo "xauth add \$(runuser -l $USERNAME -c \"xauth list \$DISPLAY\") || echo 'xauth not set; restart terminal to run simulation'" >> ~/.bashrc

RUN sudo useradd -m -p $(openssl passwd -1 "$PASSWORD") $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod -s /bin/bash $USERNAME