# Container development (layer 12)

This folder holds the **ROS 2 Humble** image and in-container scripts. Numbered folders in this repo rise in abstraction (`01_` … `11_`); deployment and cross-platform dev tooling live here as the top layer.

**Native Linux is unchanged:** on the host, `./scripts/build.sh` still targets `build/`, `install/`, and `log/` (defaults when `VEXU_COLCON_*` is unset). Only `docker-compose.yml` injects `build-docker` / `install-docker` / `log-docker` for containers.

The image bakes in Ghost `.deb` libraries (CasADi, BT.CPP, …) and a `rosdep install` pass so each `docker compose run --rm …` can compile without re-running init. Rebuild the image (`docker compose build`) after pulling major dependency or `12_Docker/Dockerfile` changes.

## Prerequisites

- Docker (Docker Desktop on Mac/Windows, or Docker Engine on Linux).
- On the **host**, clone the repo and initialize submodules (SSH keys or HTTPS must work on the host):

```bash
git clone <your-fork-or-upstream-url> VEXU_GHOST
cd VEXU_GHOST
git submodule update --init --recursive
```

## One-time image build

From the repository root:

```bash
docker compose build
# or: ./scripts/docker/build_image.sh
```

`vexu-init-workspace.sh` and `vexu-install-ghost-debs.sh` in the image are thin wrappers that **run `/vexu/12_Docker/*.sh` from your bind-mounted repo**, so you can edit those scripts without rebuilding the image. You still need `docker compose build` after changing `12_Docker/Dockerfile` or `12_Docker/entrypoint.sh`.

## Troubleshooting: `rosdep update` / “Name or service not known”

If `vexu-init-workspace.sh` fails when fetching `raw.githubusercontent.com`, Docker’s DNS is often the cause. This repo’s `docker-compose.yml` sets public DNS servers (`8.8.8.8`, etc.) on the `vexu` service to avoid that.

If it still fails, try on the host: `docker compose run --rm vexu getent hosts raw.githubusercontent.com`. No address means fix host/VPN/firewall or add DNS in Docker Desktop → Settings → Docker Engine.

Init installs **Ghost .deb packages first** (CasADi, IPOPT, …), then runs `rosdep`. While `rosdep install` runs, host **`build/` and `install/`** are moved aside briefly so broken native colcon trees do not confuse `rospack` (Docker outputs stay in `build-docker` / `install-docker`). If a run is interrupted, remove stray `.vexu-docker-stashed-*` dirs or move them back by hand. If `rosdep` fails for other reasons, retry using the command printed in the error message.

## One-time dependency install inside the container

Mounts your working tree at `/vexu` and installs `rosdep` keys plus Ghost prebuilt `.deb` packages (CasADi, IPOPT, BehaviorTree, etc.):

```bash
docker compose run --rm vexu vexu-init-workspace.sh
# or: ./scripts/docker/init_workspace.sh
```

## Build the workspace

`docker-compose.yml` sets `VEXU_COLCON_*` so colcon outputs go to `build-docker/`, `install-docker/`, and `log-docker/` (separate CMake paths from native `build/` on the same checkout).

```bash
docker compose run --rm vexu bash -lc './scripts/build.sh'
```

### Quick check that `install-docker` is on the overlay

Do **not** pipe `ros2 pkg list` into `head`: when `head` closes the pipe early, `ros2` can exit with `BrokenPipeError` even though sourcing worked.

```bash
docker compose run --rm vexu bash -lc 'source install-docker/setup.bash && ros2 pkg prefix ghost_msgs && echo OK'
```

`VEXU_IN_DOCKER` is set by the image entrypoint and skips `systemctl`/`pkill` hooks meant for the physical robot PC.

`./scripts/build.sh` stops after **colcon** in Docker (no PROS upload). Native Ubuntu still runs PROS when `pros` is installed. To force PROS inside Docker, install `gcc-arm-none-eabi` (and friends) in the image and set `VEXU_BUILD_PROS=1`, or run `./scripts/build.sh -r` on the host when you only want ROS.

If **`ghost_sim` fails with `Killed signal terminated program cc1plus`**, raise Docker Desktop **memory limit** (RAM) and retry; that is the compiler being OOM-killed.

## Interactive shell (debugging)

```bash
docker compose run --rm vexu bash
# or: ./scripts/docker/shell.sh
```

Use `gdb`, `colcon build`, `ros2 launch`, etc., as on a native 22.04 machine.

## GUI (RViz / Gazebo)

RViz and Gazebo need X11 or another display server. Examples:

- **Linux host:** `xhost +local:docker` then:

  ```bash
  docker compose run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix vexu bash
  ```

- **macOS:** install [XQuartz](https://www.xquartz.org/), allow network connections, then set `DISPLAY` to your host IP (see also `SetupMyEnvironment.md` for WSL/X11 patterns).

Apple Silicon containers report `aarch64`; the stock `build.sh` skips the simulator packages on that architecture (same as Jetson). Use an x86_64 Linux machine or VM for full Gazebo-in-Docker if you need it.

## PROS / V5 upload

The image includes `pros-cli`. USB passthrough to containers is limited on macOS; plan to run `pros upload` from Linux or from the host with a native PROS install when hardware is attached.

## VS Code / Cursor Dev Containers

Open the repo and choose “Reopen in Container”. The `.devcontainer` configuration uses `12_Docker/Dockerfile`.

## NVIDIA (optional)

On Linux with the NVIDIA Container Toolkit:

```bash
docker compose run --rm --gpus all vexu bash
```
