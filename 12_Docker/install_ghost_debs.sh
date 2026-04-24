#!/usr/bin/env bash
# Install prebuilt Ghost .deb dependencies (CasADi, IPOPT, BT.CPP, etc.).
# Root-friendly copy of scripts/setup_submodules.sh without sudo/git submodule.

set -euo pipefail

exit_unsupported_pkg() {
  echo "Failure: Unsupported package: $1. Notify maintainers of ghost_dependencies."
  exit 1
}

exit_unsupported_arch() {
  echo "Failure: Unsupported architecture: $1. ghost_dependencies provides amd64/arm64 only."
  exit 1
}

install_submodule_deb() {
  local pkg=$1
  local supported_pkgs=(
    casadi ipopt matplotlibcpp mumps rplidar btcpp btros2 btros2-interfaces
  )
  local a
  local ok=0
  for a in "${supported_pkgs[@]}"; do
    if [[ "$a" == "$pkg" ]]; then ok=1; break; fi
  done
  if [[ "$ok" -ne 1 ]]; then exit_unsupported_pkg "$pkg"; fi

  local arch
  arch=$(dpkg --print-architecture)
  if [[ "$arch" != "amd64" && "$arch" != "arm64" ]]; then
    exit_unsupported_arch "$arch"
  fi

  # TODO: publish ghost_dependencies as a real apt repo (signed Release + GPG key)
  # so this becomes `apt-get install ghost-<pkg>` — drops the wget/dpkg dance and
  # gives us versioning, signature verification, and automatic dependency resolution.
  local deb="ghost-${pkg}-${arch}.deb"
  wget -q "https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/${deb}" -O "$deb"
  dpkg -i "$deb" || apt-get install -f -y
  rm -f "$deb"
}

cd "${VEXU_HOME:?VEXU_HOME must be set}"

# Dockerfile build sets VEXU_SKIP_APT=1: the base image already installed these and the lists are fresh.
# Manual re-runs via vexu-install-ghost-debs.sh still need this.
if [ -z "${VEXU_SKIP_APT:-}" ]; then
  apt-get update
  apt-get install -y gfortran-10 liblapack-dev pkg-config swig wget ca-certificates
fi

export FC
FC=$(command -v gfortran-10)

echo "--------------- MATPLOTLIB_CPP ---------------"
install_submodule_deb matplotlibcpp

echo "--------------- MUMPS ---------------"
install_submodule_deb mumps

echo "--------------- IPOPT ---------------"
install_submodule_deb ipopt

echo "--------------- CASADI ---------------"
install_submodule_deb casadi

echo "--------------- RPLIDAR ---------------"
install_submodule_deb rplidar

echo "--------------- BTCPP ---------------"
install_submodule_deb btcpp

echo "--------------- BTROS2 ---------------"
install_submodule_deb btros2

echo "--------------- BTROS2-INTERFACES ---------------"
install_submodule_deb btros2-interfaces

# Do not delete /var/lib/apt/lists/* here: rosdep runs apt-get install next and needs the index.
apt-get clean
