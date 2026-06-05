# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Prerequisites
Ubuntu 22.04.

**For new team members doing Software Onboarding I, please start with** [Setting Up My Environment](https://github.com/VEXU-GHOST/VEXU_GHOST/blob/develop/SetupMyEnvironment.md).

## Installation
### Install ROS2 Humble

Follow Link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Repo Setup
#### Download Repository
```sh
cd
```
```
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
```
```
cd VEXU_GHOST
```
```
git submodule init
```
```
git submodule update --recursive
```
#### Add Setup to ~/.bashrc (which "configures" a new terminal when you open it)
```sh
echo "export VEXU_HOME=\"/home/$(whoami)/VEXU_GHOST\"" >> ~/.bashrc
echo 'source "$VEXU_HOME/scripts/setup_env.sh"' >> ~/.bashrc
```
Close this terminal, and open a new one to load new settings.
#### Go to the VEXU_GHOST directory (every time you open a new terminal)

```
cd ~/VEXU_GHOST
```

#### Update Dependencies
```sh
./scripts/update_dependencies.sh
```

#### Build Submodules
```sh
./scripts/setup_submodules.sh
```

#### Build Repository
```sh
./scripts/ghost.sh build
```

#### Start Simulator
```sh
./scripts/launch_sim.sh
```

#### Add yourself to the dialout group (only needed for real robot)

```sh
sudo usermod -a -G dialout $USER
```

# The `ghost` command

`scripts/ghost.sh` is the single entry point for building the code and operating the robot. Most subcommands only work once the machine has been provisioned as a robot (a name written to `/etc/ghost/robot_name`); off-robot only `run`, `build`, `clean`, and `set-robot-name` are available.

## Initial Jetson setup (before the `ghost` symlink exists)

On a fresh Jetson the `ghost` shortcut isn't installed yet, so call the script by its path. From `~/VEXU_GHOST`:

```sh
./scripts/ghost.sh set-robot-name alpha   # name this robot (writes /etc/ghost/robot_name)
./scripts/ghost.sh configure-os           # one-time OS setup; also runs install and creates the `ghost` symlink
```

After `configure-os`, the `ghost` command is available system-wide (symlinked into `/usr/local/bin`), so you can drop the `./scripts/ghost.sh` prefix and just run `ghost <command>`.

## Key usage

- `ghost run` — stops anything running and launches the hardware stack in the current terminal (foreground).
- `ghost build [pkg...]` — builds the workspace; pass package names to build only those (and their dependencies).
- `ghost clean` — removes the `build/`, `install/`, and `log/` directories.
- `ghost start` — starts the `ghost` systemd services in the background.
- `ghost restart` — restarts the `ghost` systemd services.
- `ghost stop` — stops the services and any lingering ROS/Gazebo processes.
- `ghost kill` — force-kills the services and any lingering ROS/Gazebo processes.
- `ghost shutdown` — stops the services and powers the robot off.
- `ghost install` — links and enables the `ghost` systemd services.
- `ghost configure-os` — runs the one-time Jetson OS setup (autologin, time sync, sudoers, `ghost` symlink) and `install`.
- `ghost set-robot-name <name>` — provisions this machine as a robot by writing its name to `/etc/ghost/robot_name`.

# Networking

## Connect the robot to WiFi (uplink)

```sh
sudo nmcli dev wifi connect "Velocity Wi-Fi" password "<wifi-password>"
```

## Wired ROS network (plug-in-and-go)

`ghost configure-os` sets up the robot's ethernet port (`enP8p1s0`) as a shared connection: the robot is pinned to `192.168.50.1` and runs a DHCP server on that subnet. So to connect a laptop for visualization, just **plug an ethernet cable from the laptop into the robot** — the laptop auto-gets an address (no laptop-side network config). Then run rviz:

```sh
rviz2
```
