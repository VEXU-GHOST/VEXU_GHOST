#!/bin/sh

set -x

# Resolve through symlinks so paths point at the repo. ghost.sh lives one dir up.
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
GHOST="$(cd "$DIR/.." && pwd)/ghost.sh"

# Install/enable the systemd services as part of OS setup.
"$GHOST" install

sudo mkdir -p /etc/systemd/system/serial-getty@ttyTCU0.service.d
sudo tee /etc/systemd/system/serial-getty@ttyTCU0.service.d/override.conf <<EOF > /dev/null
[Service]
# The '-o' option value tells agetty to replace 'login' arguments with an
# option to preserve environment (-p), followed by '--' for safety, and then
# the entered username.
ExecStart=
ExecStart=-/sbin/agetty --autologin ghost --keep-baud 115200,57600,38400,9600 %I $TERM
EOF
sudo systemctl daemon-reload
sudo systemctl restart serial-getty@ttyTCU0.service
sudo systemctl disable --now gdm3
echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/nopasswd-$USER

# Tighten NTP time sync via a drop-in (60- prefix = local admin range).
sudo mkdir -p /etc/systemd/timesyncd.conf.d
sudo tee /etc/systemd/timesyncd.conf.d/60-ghost.conf <<EOF > /dev/null
[Time]
ConnectionRetrySec=5
PollIntervalMinSec=3
PollIntervalMaxSec=25
EOF
sudo systemctl restart systemd-timesyncd.service

# Expose this script as the 'ghost' command system-wide.
sudo ln -sf "$GHOST" /usr/local/bin/ghost

#cd /tmp

## Download the zip archive of the repo
#wget -O jetson-orin-librealsense.zip https://codeload.github.com/jetsonhacks/jetson-orin-librealsense/zip/c8c2096b86fde54b9203107a474a07e0092a171d
#
## Unzip into a specific directory
#unzip jetson-orin-librealsense.zip -d jetson-orin-librealsense
#
## Enter the extracted subdirectory (it will be named jetson-orin-librealsense-[hash])
#cd jetson-orin-librealsense/*
## This will match: /tmp/jetson-orin-librealsense/jetson-orin-librealsense-c8c2096b86fde...
#./install-udev.sh
#
## Extract the kernel module tarball
#tar -xzf install-modules.tar.gz
#
## Enter the extracted kernel module directory
#cd install-modules
#
## Run the installation script with root privileges
#sudo ./install-realsense-modules.sh

# make sure you set the robot name with: ghost set-robot-name <name>
# (writes /etc/ghost/robot_name; same name as needed in hardware.launch.py, ex: alpha)
