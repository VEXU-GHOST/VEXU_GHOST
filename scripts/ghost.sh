#!/bin/sh

# Use this script as 'ghost' it should be installed onto your system.

set -x

# Resolve through symlinks (e.g. /usr/local/bin/ghost) so paths point at the repo.
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# A machine is a robot once its name is written to /etc/ghost/robot_name (see the
# set-robot-name command). If that's unset this isn't a robot, so only run, kill,
# build, and set-robot-name are available.
if [ ! -s /etc/ghost/robot_name ]; then
    case "$1" in
        run|kill|build|clean|set-robot-name) ;;
        *)
            echo "ghost $1: no robot name set (/etc/ghost/robot_name); only run/kill/build/clean/set-robot-name are available off-robot"
            exit 1
            ;;
    esac
fi

case "$1" in
    "start")
        echo "Starting ghost services"
        systemctl --user start --all 'ghost*.service'
        ;;
    "restart")
        echo "Restarting ghost services"
        systemctl --user kill 'ghost*service'
        systemctl --user start --all 'ghost*.service'
        ;;
    "stop")
        echo "Stopping ghost services"
        systemctl --user stop 'ghost*service'
        pkill -f -e /ros
        pkill -f -e ros2
        pkill -f -e gz
        ;;
    "kill")
        echo "Killing ghost services"
        systemctl --user kill 'ghost*service'
        pkill -f -9 -e ros
        pkill -f -9 -e ros2
        pkill -f -9 -e gz
        ;;
    "shutdown")
        echo "Shutting down ghost services"
        sudo -v
        systemctl --user stop 'ghost*service'
        pkill -f -e ros2
        echo "shutting down in 3 seconds!!!!"
        sleep 6
        sudo poweroff
        ;;
    "run")
        echo "Running ghost hardware in this terminal"
        # Stop anything already running, then launch hardware in this terminal.
        "$0" stop
        exec "$DIR/hardware/launch_hardware.sh"
        ;;
    "build")
        echo "Building ghost"
        # Pass any remaining args (package names) through to the build script.
        shift
        exec "$DIR/build/build.sh" "$@"
        ;;
    "clean")
        echo "Cleaning ghost build artifacts"
        exec "$DIR/build/clean.sh"
        ;;
    "install")
        echo "Installing ghost services"
        systemctl --user link $DIR/hardware/*service
        systemctl --user enable --now $DIR/hardware/*service
        ;;
    "configure-os")
        echo "Configuring OS for ghost"
        "$DIR/hardware/configure-os.sh"
        ;;
    "set-robot-name")
        if [ -z "$2" ]; then
            echo "Usage: ghost set-robot-name <name>"
            exit 1
        fi
        echo "Setting robot name to '$2'"
        sudo mkdir -p /etc/ghost
        echo "$2" | sudo tee /etc/ghost/robot_name > /dev/null
        ;;
    *)
        echo "Usage: ghost [run/build/clean/start/restart/stop/kill/shutdown/install/configure-os/set-robot-name <name>]"
        ;;
esac


