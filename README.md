# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Prerequisites
Ubuntu 22.04.

**For new team members doing Software Onboarding I, please start with** [Setting Up My Environment](https://github.com/VEXU-GHOST/VEXU_GHOST/blob/develop/SetupMyEnvironment.md).

## Docker (macOS / Windows / optional Linux)

For a full ROS 2 Humble environment without a native Ubuntu install, see [12_Docker/README.md](12_Docker/README.md). The image bakes in rosdep + Ghost `.deb` packages, so there is no in-container init step. Colcon writes to the usual `build/`, `install/`, `log/` in the bind-mounted checkout; the container runs as your host UID so those stay writable from the host.

Quick reference (run from the repo root):

```bash
docker compose build              # one-time: build the image
docker compose up -d              # start the long-lived dev container
docker compose exec vexu bash     # open a shell (repeat for more terminals)
# inside the shell:
./scripts/build.sh
# when done:
docker compose down               # stop + remove the container
```

`docker compose exec` reuses the same container, so background processes (e.g. `gz sim`) started in one shell stay alive for others. `up -d` also brings up a noVNC service for browser-based RViz/Gazebo (open `http://localhost:8080/vnc.html`); Linux users with a native X server can `docker compose up -d vexu` to skip it. See [12_Docker/README.md](12_Docker/README.md) for GUI, NVIDIA, PROS, and dev container details.

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
./scripts/build.sh
```

#### Start Simulator
```sh
./scripts/launch_sim.sh
```

#### Add yourself to the dialout group (only needed for real robot)

```sh
sudo usermod -a -G dialout $USER
```
