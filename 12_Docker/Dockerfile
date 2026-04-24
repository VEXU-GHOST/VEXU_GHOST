# ROS 2 Humble development image for VEXU_GHOST (Ubuntu 22.04 / Jammy).
# Build from repository root: docker compose build
#
# Ghost .deb packages (CasADi, BT.CPP, …) and rosdep system deps are installed here so
# `docker compose run --rm` is self-contained — no in-container init step required.
# Rebuild the image (`docker compose build`) after changing dependency declarations or
# bumping Ghost .debs.

FROM osrf/ros:humble-desktop-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV VEXU_HOME=/vexu
ENV VEXU_IN_DOCKER=1
ENV ROS_DISTRO=humble
ENV PROS_CLI_VERSION=3.5.6
# Single-container DDS isolation (avoids multicast issues on Docker Desktop / macOS).
# Compose may override.
ENV ROS_LOCALHOST_ONLY=1
ENV LD_LIBRARY_PATH=/usr/local/lib/
ENV GAZEBO_PLUGIN_PATH=/vexu/build/ghost_sim
WORKDIR /vexu

# One apt layer: base toolchain + what install_ghost_debs.sh and rosdep will need. Lists are
# kept until the final rosdep install so no apt-get update needs to run again during build.
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
    liblapack-dev \
    pkg-config \
    swig \
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
 && pip3 install --no-cache-dir --upgrade pip \
 && pip3 install --no-cache-dir \
    colcon-lint \
    setuptools==61 \
    piper-tts==1.2.0 \
    "pros-cli==${PROS_CLI_VERSION}" \
 && rosdep init || true

# Ghost-provided .debs (CasADi, IPOPT, BT.CPP, …). Script's apt-get update is skipped here
# because the build above already refreshed lists; runtime callers still run it.
COPY 12_Docker/install_ghost_debs.sh /usr/local/share/vexu/install_ghost_debs.sh
RUN chmod +x /usr/local/share/vexu/install_ghost_debs.sh && \
    mkdir -p /vexu && \
    VEXU_HOME=/vexu VEXU_SKIP_APT=1 bash /usr/local/share/vexu/install_ghost_debs.sh

# Snapshot workspace for rosdep only; bind-mount replaces /vexu at runtime.
# Code changes will re-run rosdep install, but it's fast once apt packages are already installed.
COPY . /vexu
RUN rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y \
      --skip-keys behaviortree_cpp \
      --skip-keys behaviortree_ros2 \
      --skip-keys btcpp_ros2_interfaces \
      --skip-keys btcpp_ros2_samples \
      --skip-keys rplidar_ros \
      --skip-keys librealsense2 \
      --skip-keys realsense2_camera && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY 12_Docker/entrypoint.sh /usr/local/bin/vexu-entrypoint.sh
# Manual re-run wrapper (bind-mounted repo so edits to install_ghost_debs.sh apply
# without rebuilding the image). Use after Ghost publishes new .debs.
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -eo pipefail' \
    'exec bash /vexu/12_Docker/install_ghost_debs.sh "$@"' \
    > /usr/local/bin/vexu-install-ghost-debs.sh && \
    chmod +x /usr/local/bin/vexu-entrypoint.sh /usr/local/bin/vexu-install-ghost-debs.sh

ENTRYPOINT ["/usr/local/bin/vexu-entrypoint.sh"]
CMD ["bash"]
