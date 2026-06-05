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

# --- Dedicated wired ROS network: the robot is the DHCP server + gateway ---
# (Inspired by innate-os.) 'ipv4.method shared' pins the interface to a fixed IP
# AND runs a dnsmasq DHCP server + NAT on the subnet, so a laptop just plugs in
# the cable and auto-gets an address (zero config on the remote).
#
# 'ipv4.never-default yes' keeps WiFi as the robot's default route, so the robot
# resolves DNS via WiFi's upstream; shared-mode dnsmasq forwards client DNS to
# that same upstream. This matters for captive portals: once the robot logs in
# over WiFi, the NAT'd clients ride its authenticated session and resolve through
# the portal's DNS too.
ETH_CONNECTION="ghost-eth"
ETH_INTERFACE="enP8p1s0"
ETH_IP="192.168.50.1/24"
if ip link show "$ETH_INTERFACE" >/dev/null 2>&1; then
	sudo nmcli dev set "$ETH_INTERFACE" managed yes 2>/dev/null || true
	if nmcli con show "$ETH_CONNECTION" >/dev/null 2>&1; then
		sudo nmcli con modify "$ETH_CONNECTION" \
			connection.interface-name "$ETH_INTERFACE" \
			connection.autoconnect yes connection.autoconnect-priority 10 \
			ipv4.method shared ipv4.addresses "$ETH_IP" \
			ipv4.never-default yes
	else
		sudo nmcli con add type ethernet ifname "$ETH_INTERFACE" con-name "$ETH_CONNECTION" \
			connection.autoconnect yes connection.autoconnect-priority 10 \
			ipv4.method shared ipv4.addresses "$ETH_IP" \
			ipv4.never-default yes
	fi
	sudo nmcli con up "$ETH_CONNECTION" || echo "ghost-eth: cable not plugged in yet; profile will auto-activate later"
else
	echo "ghost-eth: interface $ETH_INTERFACE not found; skipping wired ROS network"
fi

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
