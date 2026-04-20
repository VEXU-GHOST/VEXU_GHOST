# ROS 2 Humble development image for VEXU_GHOST (Ubuntu 22.04 / Jammy).
# Build from repository root: docker compose build
#
# Ghost .deb packages (CasADi, BT.CPP, …) and rosdep system deps are installed here so
# `docker compose run --rm` is self-contained. (Ephemeral containers do not keep packages
# from a previous `vexu-init-workspace.sh` run.)

FROM osrf/ros:humble-desktop-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV VEXU_HOME=/vexu
ENV VEXU_IN_DOCKER=1
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    ca-certificates \
    openssh-client \
    ccache \
    cmake \
    build-essential \
    gdb \
    gdbserver \
    valgrind \
    vim \
    less \
    libgoogle-glog-dev \
    libgtest-dev \
    gfortran-10 \
    libi2c-dev \
    libi2c0 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    apt-rdepends \
    ros-humble-xacro \
    sox \
    libsox-fmt-mp3 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    colcon-lint \
    setuptools==61 \
    piper-tts==1.2.0 \
    pros-cli

# Pre-install Ghost-provided .debs (same as 12_Docker/install_ghost_debs.sh at build time).
COPY 12_Docker/install_ghost_debs.sh /usr/local/share/vexu/install_ghost_debs.sh
RUN chmod +x /usr/local/share/vexu/install_ghost_debs.sh && \
    mkdir -p /vexu && \
    VEXU_HOME=/vexu bash /usr/local/share/vexu/install_ghost_debs.sh

# Snapshot workspace for rosdep only; bind-mount replaces /vexu at runtime but /usr stays.
COPY . /vexu
WORKDIR /vexu
RUN apt-get update && \
    apt-get remove -y 'ros-humble-rplidar-ros*' 'ros-humble-behaviortree-cpp*' 2>/dev/null || true && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths . --ignore-src -r -y \
      --skip-keys behaviortree_cpp \
      --skip-keys behaviortree_ros2 \
      --skip-keys btcpp_ros2_interfaces \
      --skip-keys btcpp_ros2_samples \
      --skip-keys rplidar_ros \
      --skip-keys librealsense2 \
      --skip-keys realsense2_camera && \
    rm -rf /var/lib/apt/lists/*

COPY 12_Docker/entrypoint.sh /usr/local/bin/vexu-entrypoint.sh
# Wrapper scripts use the bind-mounted repo so edits apply without rebuilding (except deps above).
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -eo pipefail' \
    'exec bash /vexu/12_Docker/init_workspace.sh "$@"' \
    > /usr/local/bin/vexu-init-workspace.sh && \
    printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -eo pipefail' \
    'exec bash /vexu/12_Docker/install_ghost_debs.sh "$@"' \
    > /usr/local/bin/vexu-install-ghost-debs.sh && \
    chmod +x /usr/local/bin/vexu-entrypoint.sh /usr/local/bin/vexu-init-workspace.sh /usr/local/bin/vexu-install-ghost-debs.sh

WORKDIR /vexu

ENTRYPOINT ["/usr/local/bin/vexu-entrypoint.sh"]
CMD ["bash"]
