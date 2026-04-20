#!/usr/bin/env bash
# First-time (or after dependency changes): Ghost .debs + rosdep inside the container.
set -euo pipefail
export VEXU_HOME="${VEXU_HOME:-/vexu}"
cd "$VEXU_HOME"

apt-get update
apt-get remove -y 'ros-humble-rplidar-ros*' 'ros-humble-behaviortree-cpp*' 2>/dev/null || true

echo "--------------- Ghost prebuilt packages (CasADi, BT, …) ---------------"
bash "$VEXU_HOME/12_Docker/install_ghost_debs.sh"

echo "--------------- rosdep update / install ---------------"
rosdep_update_ok=0
for attempt in 1 2 3 4 5; do
  if rosdep update; then
    rosdep_update_ok=1
    break
  fi
  echo "rosdep update failed (attempt $attempt/5). Retrying in 8s…"
  sleep 8
done

if [[ "$rosdep_update_ok" -ne 1 ]]; then
  echo >&2 ""
  echo >&2 "ERROR: rosdep update could not reach rosdistro (e.g. raw.githubusercontent.com)."
  echo >&2 "  • Try again: your network or Docker DNS may have been flaky."
  echo >&2 "  • This compose file sets public DNS (8.8.8.8, 1.1.1.1); ensure outbound HTTPS is allowed."
  echo >&2 "Ghost .deb packages are already installed; you can retry only rosdep later:"
  echo >&2 "  docker compose run --rm vexu bash -lc 'cd /vexu && rosdep update && rosdep install --from-paths . --ignore-src -r -y --skip-keys behaviortree_cpp behaviortree_ros2 btcpp_ros2_interfaces btcpp_ros2_samples rplidar_ros librealsense2 realsense2_camera'"
  exit 1
fi

# Bind-mounted host ./install and ./build are often partial colcon trees; rospack/rosdep then
# errors on missing package.xml. Hide them only for rosdep install (not build-docker / install-docker).
STASH_INSTALL="$VEXU_HOME/.vexu-docker-stashed-install"
STASH_BUILD="$VEXU_HOME/.vexu-docker-stashed-build"
stashed_install=0
stashed_build=0

restore_stashed_workspace_dirs() {
  if [[ "$stashed_build" -eq 1 ]] && [[ -d "$STASH_BUILD" ]]; then
    mv "$STASH_BUILD" "$VEXU_HOME/build"
    stashed_build=0
  fi
  if [[ "$stashed_install" -eq 1 ]] && [[ -d "$STASH_INSTALL" ]]; then
    mv "$STASH_INSTALL" "$VEXU_HOME/install"
    stashed_install=0
  fi
}
trap restore_stashed_workspace_dirs EXIT

if [[ -d "$VEXU_HOME/build" ]]; then
  echo "--------------- stash host ./build during rosdep (partial colcon trees confuse rospack) ---------------"
  rm -rf "$STASH_BUILD"
  mv "$VEXU_HOME/build" "$STASH_BUILD"
  stashed_build=1
fi

if [[ -d "$VEXU_HOME/install" ]]; then
  echo "--------------- stash host ./install during rosdep (partial colcon trees confuse rospack) ---------------"
  rm -rf "$STASH_INSTALL"
  mv "$VEXU_HOME/install" "$STASH_INSTALL"
  stashed_install=1
fi

# Drop overlay env from the host shell / old setup.bash; keep only Humble underlay for rosdep.
set +u
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_WORKSPACE || true
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u

# Refresh index after install_ghost_debs.sh (and ensure universe is available for Qt, lz4, etc.).
apt-get update

# Skip keys satisfied by Ghost .deb packages or omitted in scripts/build.sh (--packages-skip).
rosdep install --from-paths . --ignore-src -r -y \
  --skip-keys behaviortree_cpp \
  --skip-keys behaviortree_ros2 \
  --skip-keys btcpp_ros2_interfaces \
  --skip-keys btcpp_ros2_samples \
  --skip-keys rplidar_ros \
  --skip-keys librealsense2 \
  --skip-keys realsense2_camera

restore_stashed_workspace_dirs
trap - EXIT

echo "Done. You can run: ./scripts/build.sh"
