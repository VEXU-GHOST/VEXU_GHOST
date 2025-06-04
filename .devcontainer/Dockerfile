FROM ubuntu:22.04

RUN apt update
RUN apt install sudo -y

# Install and set locale
RUN apt update
RUN apt install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Enable Ubuntu Universe repository
RUN apt install software-properties-common -y
RUN add-apt-repository universe

# Add ROS2 GPG key
RUN apt update
RUN apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
RUN apt update
RUN apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt install ros-humble-desktop -y

RUN apt install python3-colcon-common-extensions -y


# Set up submodules
RUN apt update
RUN apt install wget
COPY scripts/setup_submodules.sh /tmp/setup_submodules.sh
RUN /tmp/setup_submodules.sh

RUN apt install libgoogle-glog-dev -y

# Create ghost user
RUN useradd -rm -d /home/ghost -s /bin/bash -g root -G sudo -u 1000 ghost
RUN echo "ghost ALL=(ALL:ALL) NOPASSWD: ALL" > "/etc/sudoers.d/dont-prompt-ghost-for-sudo-password"
USER ghost
WORKDIR /home/ghost

# Add setup to .bashrc
RUN echo 'export VEXU_HOME=/workspaces/VEXU_GHOST' >> ~/.bashrc
RUN echo 'source /opt/ros/humble/setup.sh' >> ~/.bashrc
