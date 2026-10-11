#!/usr/bin/env bash
# Install/check standalone MVP packages, independently of robot startup/builds.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash setup_dependencies.sh [--check | --install | --versions] [--camera]

  --check     Check selected packages and detector version (default; read-only).
  --install   Install selected packages using the configured Ubuntu/ROS apt repos.
  --versions  Print installed package inventory as TSV (read-only).
  --camera    Include RealSense packages; omit for detector/bag-replay-only setup.

Requires Ubuntu 22.04, amd64 or arm64. For installation, first configure the ROS
apt repository using the official Humble installation guide linked in README.md.
Installation uses sudo when needed. It never starts ROS nodes or robot services.
EOF
}

mode=check
mode_set=false
camera=false
for arg in "$@"; do
  case "$arg" in
    --check|--install|--versions)
      if "$mode_set"; then
        echo 'Choose only one of --check, --install, or --versions.' >&2
        exit 2
      fi
      mode=${arg#--}
      mode_set=true
      ;;
    --camera) camera=true ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  echo 'Run this script in Ubuntu 22.04, for example through WSL 2.' >&2
  exit 2
fi
source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 22.04 ]]; then
  echo "Expected Ubuntu 22.04; found ${PRETTY_NAME:-unknown OS}." >&2
  exit 2
fi
arch=$(dpkg --print-architecture)
if [[ $arch != amd64 && $arch != arm64 ]]; then
  echo "Unsupported architecture: $arch (expected amd64 or arm64)." >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
manifests=("$script_dir/packages-core.txt")
if "$camera"; then
  manifests+=("$script_dir/packages-camera.txt")
fi
packages=()
for manifest in "${manifests[@]}"; do
  while IFS= read -r line || [[ -n $line ]]; do
    line=${line%$'\r'}
    [[ -z $line || $line == \#* ]] && continue
    if [[ ! $line =~ ^[a-z0-9][a-z0-9+.-]+$ ]]; then
      echo "Invalid package name in $manifest: $line" >&2
      exit 2
    fi
    packages+=("$line")
  done < "$manifest"
done

if [[ $mode == install ]]; then
  elevate=()
  if [[ $EUID -ne 0 ]]; then
    elevate=(sudo)
  fi
  "${elevate[@]}" apt-get update
  # Fail clearly before attempting a partial install if ROS sources are missing.
  for package in "${packages[@]}"; do
    candidate=$(LC_ALL=C apt-cache policy "$package" | awk '/Candidate:/ {print $2}')
    if [[ -z $candidate || $candidate == '(none)' ]]; then
      echo "No apt candidate for $package on $arch." >&2
      echo 'Configure the official Humble apt repository first; see README.md.' >&2
      exit 1
    fi
  done
  detector_candidate=$(LC_ALL=C apt-cache policy ros-humble-apriltag-ros | awk '/Candidate:/ {print $2}')
  if ! dpkg --compare-versions "$detector_candidate" ge 3.4.0; then
    echo "apriltag_ros >= 3.4.0 required; apt candidate is $detector_candidate." >&2
    exit 1
  fi
  # --no-remove prevents apt from removing existing packages to satisfy this request.
  # Leave apt's normal confirmation visible so the user can review its transaction.
  "${elevate[@]}" apt-get install --no-remove "${packages[@]}"
fi

failed=0
if [[ $mode == versions ]]; then
  printf 'package\tversion\tarchitecture\n'
fi
for package in "${packages[@]}"; do
  status=$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)
  if [[ $status != 'install ok installed' ]]; then
    echo "Missing package: $package" >&2
    failed=1
    continue
  fi
  version=$(dpkg-query -W -f='${Version}' "$package")
  if [[ $mode == versions ]]; then
    dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' "$package"
  else
    printf 'Installed: %s %s\n' "$package" "$version"
  fi
  if [[ $package == ros-humble-apriltag-ros ]] && ! dpkg --compare-versions "$version" ge 3.4.0; then
    echo "apriltag_ros >= 3.4.0 required; installed: $version" >&2
    failed=1
  fi
done
if [[ $failed -ne 0 ]]; then
  exit 1
fi
if [[ $mode != versions ]]; then
  echo 'Package check passed. Source /opt/ros/humble/setup.bash before launching.'
  echo 'This checks installation only; camera access and ROS detection need runtime tests.'
fi
