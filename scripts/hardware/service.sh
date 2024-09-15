#!/bin/sh

set -x

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$1" in
    "restart")
        systemctl --user kill 'ghost*service'
        systemctl --user start 'ghost*.service'
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
        systemctl --user stop 'ghost*service'
        pkill -f -e ros2
        echo "shutting down in 3 seconds!!!!"
        sleep 6
        sudo poweroff
        ;;
    "install")
        echo systemctl --user link $DIR/*service
        echo systemctl --user enable --now $DIR/*service
        ;;
    *)
        echo "Usage: service.sh [restart/stop/kill/shutdown/install]"
        ;;
esac


