# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Setup

The recommended flow is Docker — works the same on macOS, Windows, and Linux, and you never have to match a specific Ubuntu version on the host.

**Windows / macOS users:** start at [SetupMyEnvironment.md](SetupMyEnvironment.md) for Docker Desktop + SSH/Git setup, then come back here.

## 1. Install Docker

- **Linux:** [Docker Engine](https://docs.docker.com/engine/install/) (any distro; no Ubuntu requirement).
- **macOS / Windows:** [Docker Desktop](https://www.docker.com/products/docker-desktop/).

## 2. Clone the repo (on your host, not inside a container)

```bash
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
git submodule update --init --recursive
```

SSH keys and `~/.gitconfig` on your host are bind-mounted read-only into the container, so git inside the container uses the same identity.

## 3. Build the image and start the dev container

```bash
docker compose build              # one-time; ~10–15 min on first build
docker compose up -d              # start the long-lived dev container
docker compose exec vexu bash     # open a shell (repeat for more terminals)
```

Inside the shell:

```bash
./scripts/build.sh                # compile the ROS 2 workspace
./scripts/launch_sim.sh           # start the Gazebo sim
```

When you're done:

```bash
docker compose down               # stop + remove the container (volumes persist)
```

`docker compose exec` reuses the same container — background processes like `gz sim` started in one shell stay alive for other shells. `up -d` also starts a noVNC service for browser-based RViz/Gazebo on [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html); Linux users with a native display skip it via `docker compose up -d vexu`.

See [12_Docker/README.md](12_Docker/README.md) for GUI, NVIDIA, PROS, dev-container, and implementation details.

# Native Ubuntu 22.04 (alternative)

If you prefer a native install on Ubuntu 22.04 (no Docker), the old manual flow still works:

<details>
<summary>Click to expand native setup</summary>

### Install ROS 2 Humble

Follow: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

### Repo setup

```bash
cd
git clone git@github.com:VEXU-GHOST/VEXU_GHOST.git
cd VEXU_GHOST
git submodule update --init --recursive
```

### Add setup to ~/.bashrc

```bash
echo "export VEXU_HOME=\"$HOME/VEXU_GHOST\"" >> ~/.bashrc
echo 'source "$VEXU_HOME/scripts/setup_env.sh"' >> ~/.bashrc
```

Open a new terminal to pick up the new env.

### Build

```bash
./scripts/update_dependencies.sh
./scripts/setup_submodules.sh
./scripts/build.sh
./scripts/launch_sim.sh
```

### Real robot USB access

```bash
sudo usermod -a -G dialout $USER
```

</details>
