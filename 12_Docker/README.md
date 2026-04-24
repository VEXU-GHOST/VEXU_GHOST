# Container development (layer 12)

This folder holds the **ROS 2 Humble** image and in-container scripts. Numbered folders in this repo rise in abstraction (`01_` … `11_`); deployment and cross-platform dev tooling live here as the top layer.

**Colcon outputs land in the usual `build/`, `install/`, `log/` dirs** in your bind-mounted checkout. The container runs as your host UID (via `HOST_UID`/`HOST_GID` build args), so everything is writable from the host too. Don't run `./scripts/build.sh` natively *and* in the container on the same checkout — CMake caches absolute paths and the two environments will corrupt each other.

The image bakes in Ghost `.deb` libraries (CasADi, BT.CPP, …) and a `rosdep install` pass so each `docker compose run --rm …` can compile immediately — there is no in-container init step. Rebuild the image (`docker compose build`) after pulling major dependency changes, bumping Ghost .debs, or editing `12_Docker/Dockerfile`.

## Prerequisites

- Docker (Docker Desktop on Mac/Windows, or Docker Engine on Linux).
- On the **host**, clone the repo and initialize submodules (SSH keys or HTTPS must work on the host):

```bash
git clone <your-fork-or-upstream-url> VEXU_GHOST
cd VEXU_GHOST
git submodule update --init --recursive
```

All commands below assume you are at the repository root (where [docker-compose.yml](../docker-compose.yml) lives).

**Linux users with a native X server:** `docker compose up -d` starts the bundled noVNC container by default (for macOS/Windows). You can skip it and use your host X11 instead — see [GUI (RViz / Gazebo)](#gui-rviz--gazebo).

## One-time image build

```bash
docker compose build
```

`vexu-install-ghost-debs.sh` in the image runs `/vexu/12_Docker/install_ghost_debs.sh` from your bind-mounted repo — invoke it manually if Ghost publishes new `.deb` versions and you don't want to rebuild the image. Changes to `12_Docker/Dockerfile` or `12_Docker/entrypoint.sh` still require `docker compose build`.

## Daily workflow

```bash
docker compose up -d              # start the long-lived dev container
docker compose exec vexu bash     # open a shell (repeat for more terminals)
docker compose down               # stop + remove the container when done
```

`exec` attaches to the *same* running container, so `gz sim` (or any background process) you started in shell #1 is still alive in shell #2. In-container state (running processes, shell history) is lost on `docker compose down`, but the bind-mounted repo at `/vexu` — including `build/`, `install/`, `log/` — and the `vexu-ccache` volume survive.

## Troubleshooting: `rosdep update` / “Name or service not known”

If `docker compose build` fails when fetching `raw.githubusercontent.com`, Docker’s DNS is often the cause. This repo’s `docker-compose.yml` sets public DNS servers (`8.8.8.8`, etc.) on the `vexu` service; the build also benefits if your Docker daemon respects those.

If it still fails, try on the host: `docker compose run --rm vexu getent hosts raw.githubusercontent.com`. No address means fix host/VPN/firewall or add DNS in Docker Desktop → Settings → Docker Engine.

## Build the workspace

Inside an `exec`'d shell:

```bash
./scripts/build.sh
```

Colcon writes to `build/`, `install/`, `log/` in your repo (bind-mounted). The container user matches your host UID so these stay writable from the host too — wipe with `rm -rf build install log` (no sudo needed).

### Quick check that the overlay is sourced

Do **not** pipe `ros2 pkg list` into `head`: when `head` closes the pipe early, `ros2` can exit with `BrokenPipeError` even though sourcing worked.

```bash
docker compose exec vexu bash -lc 'ros2 pkg prefix ghost_msgs && echo OK'
```

The entrypoint already sources `install/setup.bash` if it exists. `VEXU_IN_DOCKER` is set in the image and skips `systemctl`/`pkill` hooks meant for the physical robot PC.

`./scripts/build.sh` stops after **colcon** in Docker (no PROS upload). Native Ubuntu still runs PROS when `pros` is installed. To force PROS inside Docker, install `gcc-arm-none-eabi` (and friends) in the image and set `VEXU_BUILD_PROS=1`, or run `./scripts/build.sh -r` on the host when you only want ROS.

If **`ghost_sim` fails with `Killed signal terminated program cc1plus`**, raise Docker Desktop **memory limit** (RAM) and retry; that is the compiler being OOM-killed.

## One-off / ephemeral container

If you don't want a long-lived container (e.g. for CI or a single command), use `run --rm` instead — it spins up a fresh container, runs the command, and deletes it on exit:

```bash
docker compose run --rm vexu bash -lc './scripts/build.sh'
```

## GUI (RViz / Gazebo)

**Default (macOS / Windows, and any Linux user who just wants it to work):** `docker compose up -d` starts the `novnc` service alongside `vexu`. `DISPLAY` in the dev container defaults to `novnc:0.0`, so Qt / OpenGL apps render into Xvfb and stream over WebSockets.

```bash
docker compose up -d
docker compose exec vexu bash
# open http://localhost:8080/vnc.html in a browser, then run rviz2 / gz sim from the shell
```

**Native Linux X server (faster, shared-memory rendering — opt out of noVNC):**

```bash
xhost +local:docker
docker compose up -d vexu        # vexu only, no novnc
docker compose run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix vexu bash
```

On macOS you can also use [XQuartz](https://www.xquartz.org/) directly instead of noVNC; see `SetupMyEnvironment.md` for WSL/X11 patterns.

Apple Silicon containers report `aarch64`; the stock `build.sh` skips the simulator packages on that architecture (same as Jetson). Use an x86_64 Linux machine or VM for full Gazebo-in-Docker if you need it.

## PROS / V5 upload

The image includes `pros-cli`. USB passthrough to containers is limited on macOS; plan to run `pros upload` from Linux or from the host with a native PROS install when hardware is attached.

## VS Code / Cursor Dev Containers

Open the repo and choose “Reopen in Container”. The `.devcontainer` configuration reuses the `vexu` service from `docker-compose.yml`, so it inherits the same env vars, DNS, bind-mounted build paths, and ccache volume as CLI usage — no config drift.

## NVIDIA (optional)

On Linux with the NVIDIA Container Toolkit:

```bash
docker compose run --rm --gpus all vexu bash
```
