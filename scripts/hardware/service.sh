#!/bin/sh

set -x

DIR="$(cd "$(dirname "$0")" && pwd)"

case "$1" in
    "restart")
        systemctl --user kill 'ghost*service'
        systemctl --user start --all 'ghost*.service'
        ;;
    "stop")
        systemctl --user stop 'ghost*service'
        pkill -f -e ros
        pkill -f -e ros2
        pkill -f -e gz
        ;;
    "kill")
        systemctl --user kill 'ghost*service'
        pkill -f -9 -e ros
        pkill -f -9 -e ros2
        pkill -f -9 -e gz
        ;;
    "shutdown")
        sudo -v
        systemctl --user stop 'ghost*service'
        pkill -f -e ros2
        echo "shutting down in 3 seconds!!!!"
        sleep 6
        sudo poweroff
        ;;
    "install")
        systemctl --user link $DIR/*service
        systemctl --user enable --now $DIR/*service
        ;;
    "autologin")
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
	;;
    *)
        echo "Usage: service.sh [restart/stop/kill/shutdown/install]"
        ;;
esac


