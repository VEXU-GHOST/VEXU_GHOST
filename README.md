# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Setup

The recommended flow is Docker — works the same on macOS, Windows, and Linux, and you never have to match a specific Ubuntu version on the host.

**Windows / macOS users:** start at [SetupMyEnvironment.md](SetupMyEnvironment.md) for Docker Desktop + SSH/Git setup, then come back here.

> [!NOTE]
> Recommended for most users: open this repo in VS Code and use **Dev Containers: Reopen in Container**. VS Code will manage container start/attach for you and auto-install the recommended extensions from [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json). Use the CLI `docker compose ...` flow below if you prefer terminal-only control.

## 2. Clone the repo (on your host, not inside a container)

```bash
git clone git@github.com:VEXU-GHOST/VEXU-GHOST.git
cd VEXU-GHOST
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
./scripts/launch_sim.sh           # start the Gazebo sim, watch it at http://localhost:8080/vnc.html
```

When you're done:

```bash
docker compose stop               # pause the container; keep in-container filesystem state
```

`docker compose exec` reuses the same container — background processes like `gz sim` started in one shell stay alive for other shells. `up -d` also starts a noVNC service for browser-based RViz/Gazebo on [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html); Linux users with a native display skip it via `docker compose up -d vexu`.

Use `docker compose down` only when you want a reset: it removes containers and drops writable container filesystem changes (for example, tools/extensions installed inside the container). Bind-mounted repo files and named volumes still persist unless you also pass `--volumes`.

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
git clone git@github.com:VEXU-GHOST/VEXU-GHOST.git
cd VEXU-GHOST
git submodule update --init --recursive
git checkout develop # change develop to the branch you are working on
```

### Add setup to ~/.bashrc

```bash
echo "export VEXU_HOME=\"$HOME/VEXU-GHOST\"" >> ~/.bashrc
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
