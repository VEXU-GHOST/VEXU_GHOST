# Container development (layer 12)

This folder holds the **ROS 2 Humble** image and in-container scripts. Numbered folders in this repo rise in abstraction (`01_` … `11_`); deployment and cross-platform dev tooling live here as the top layer.

**Colcon outputs land in the usual `build/`, `install/`, `log/` dirs** in your bind-mounted checkout. The container runs as root, so those dirs are root-owned on the host — clean them with `sudo rm -rf build install log` or from inside the container (`docker compose exec vexu rm -rf build install log`). Don't run `./scripts/build.sh` natively *and* in the container on the same checkout — CMake caches absolute paths and the two environments will corrupt each other.

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
docker compose stop               # pause when done; keep writable container state
```

`exec` attaches to the *same* running container, so `gz sim` (or any background process) you started in shell #1 is still alive in shell #2. `docker compose stop` keeps writable in-container filesystem state (including tools/extensions installed inside the container), while `docker compose down` removes containers and drops that writable layer. The bind-mounted repo at `/vexu` — including `build/`, `install/`, `log/` — and the `vexu-ccache` volume survive unless you explicitly remove volumes.

## Troubleshooting: `rosdep update` / “Name or service not known”

If `docker compose build` fails when fetching `raw.githubusercontent.com`, Docker’s DNS is often the cause. This repo’s `docker-compose.yml` sets public DNS servers (`8.8.8.8`, etc.) on the `vexu` service; the build also benefits if your Docker daemon respects those.

If it still fails, try on the host: `docker compose run --rm vexu getent hosts raw.githubusercontent.com`. No address means fix host/VPN/firewall or add DNS in Docker Desktop → Settings → Docker Engine.

## Troubleshooting: `$'\r': command not found` / `bad interpreter`

Symptoms, any of these:

```
./scripts/build.sh: line 2: $'\r': command not found
bash: ./scripts/build.sh: /bin/bash^M: bad interpreter: No such file or directory
```

Your checkout has CRLF line endings — typically a Windows clone made before
`.gitattributes` pinned `eol=lf`, with `core.autocrlf=true`. The worktree is what
`docker-compose.yml` bind-mounts as `/vexu`, so the container sees the CRLF too.

Fix the checkout once, on the **host**, from anywhere in the repo:

```bash
bash <(git show HEAD:scripts/fix_line_endings.sh)
```

Use that form rather than `bash scripts/fix_line_endings.sh`: on a CRLF checkout that
script is itself CRLF and bash dies on its own first line. `git show` reads the blob
from the object store, which is LF no matter what the worktree looks like.

It only rewrites tracked text files whose attributes ask for LF, so binaries and the
`eol=crlf` Windows scripts are left alone. It is a no-op on a healthy checkout, and it
produces no commit — those files are already LF in the index, so this just realigns the
worktree with it. Fresh clones need none of this.

If `docker compose build` itself failed before you could get a shell, that is the same
cause: the image `COPY`s `12_Docker/entrypoint.sh` and `install_ghost_debs.sh` from your
checkout. The Dockerfile strips CR from both, so a rebuild after `git pull` will succeed
regardless — but still run the script above to fix the tree colcon compiles.

## Build the workspace

Inside an `exec`'d shell:

```bash
./scripts/build.sh
```

Colcon writes to `build/`, `install/`, `log/` in your repo (bind-mounted). Files are root-owned on the host — wipe with `sudo rm -rf build install log` or from inside the container (`rm -rf build install log`, no sudo needed there).

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

**Native Linux X server / XWayland (faster, shared-memory rendering — opt out of noVNC):**

```bash
xhost +local:root                # one-time per login: grant container-root X access
docker compose up -d vexu        # vexu only, no novnc
docker compose exec vexu bash
```

The compose file already bind-mounts `/tmp/.X11-unix` and forwards `DISPLAY`. On Ubuntu 22.04+ with GNOME (Wayland), your `DISPLAY=:0` actually points at XWayland — Qt apps use Wayland directly, Ogre-based ones (RViz) use XWayland. For GPU-accelerated rendering (Linux only), uncomment the `/dev/dri` devices block in `docker-compose.yml` and set `LIBGL_ALWAYS_SOFTWARE=0` in `.env`.

On macOS you can also use [XQuartz](https://www.xquartz.org/) directly instead of noVNC.

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

## Under the hood

A short tour of what `docker compose build` and `docker compose up -d` actually do, for when things break and you need to dig in.

### Image build ([Dockerfile](Dockerfile))

1. **Base:** `osrf/ros:humble-desktop-jammy` — Ubuntu 22.04 with ROS 2 Humble desktop pre-installed.
2. **System packages + Python (one RUN layer):** apt installs toolchain (gcc, cmake, gdb), ROS/colcon tooling, and the deps that later stages need (gfortran-10, liblapack-dev, swig, …); pip installs `pros-cli==3.5.6`, `piper-tts`, `colcon-lint`; `rosdep init` runs once. Apt lists are kept through to the final rosdep install so `apt-get update` runs exactly once per build.
3. **Ghost `.deb` packages** ([install_ghost_debs.sh](install_ghost_debs.sh)): downloads prebuilt CasADi, IPOPT, MUMPS, BT.CPP, rplidar, etc. from the `ghost_dependencies` GitHub repo and `dpkg -i`s them. Run with `VEXU_SKIP_APT=1` to skip the script's own apt refresh (the one from step 2 is still fresh).
4. **`rosdep install` on a snapshot of the workspace:** the full repo is `COPY`'d to `/vexu` just so rosdep can read every `package.xml`. Bind-mount at runtime replaces `/vexu` anyway, so the snapshot is throwaway. Code edits re-run this step; it's fast because apt packages are already installed.
5. **Shell env setup:** writes `/etc/profile.d/vexu-ros.sh` and sources it from `/etc/bash.bashrc` so every `docker compose exec vexu bash` has `ros2`, `colcon`, and the workspace overlay on PATH automatically. (The ENTRYPOINT sources the same env for `CMD`-invoked shells; exec shells bypass ENTRYPOINT, hence the bashrc hook.)
6. **Entrypoint** ([entrypoint.sh](entrypoint.sh)): sources `/opt/ros/humble/setup.bash`, then sources `$VEXU_HOME/install/setup.bash` if it exists, then `exec "$@"`.

### Runtime ([../docker-compose.yml](../docker-compose.yml))

1. **Runs as root.** Files on the host bind mount end up root-owned; accepted as a tradeoff for a much simpler image (no UID-matching, sudo, or /run/user plumbing). Clean with `sudo rm -rf build install log` or from inside the container.
2. **`command: ["sleep", "infinity"]`** — the container idles until you `docker compose exec` into it. `up -d` brings it up in the background; `down` tears it down.
3. **Bind mounts:**
   - `.:/vexu` — the repo. Colcon writes `build/`, `install/`, `log/` directly to your checkout.
   - `~/.ssh:/root/.ssh:ro`, `~/.gitconfig:/root/.gitconfig:ro` — git from inside the container uses your host identity.
   - `/tmp/.X11-unix:/tmp/.X11-unix` — XWayland/X11 socket for RViz (Ogre3D isn't Wayland-capable). Harmless no-op on macOS/Windows.
   - `${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}` → `/tmp/${WAYLAND_DISPLAY}` — Wayland socket passthrough. Sentinel default (`no-wayland.sock`) keeps the mount harmless on macOS/Windows.
4. **Rendering:** `LIBGL_ALWAYS_SOFTWARE=1` by default — Mesa renders via llvmpipe client-side instead of calling the server's GLX (which noVNC's Xvfb doesn't provide). Works everywhere, ~10x slower than GPU. For hardware rendering on Linux, uncomment the `devices: /dev/dri:/dev/dri` block and set `LIBGL_ALWAYS_SOFTWARE=0` in `.env`.
5. **Persistent state:** Named volume `vexu-ccache` at `/root/.ccache` persists compile cache across `down`/`up`. Named volume `vexu-bashhistory` at `/commandhistory` (HISTFILE points there) keeps shell history. Named volume `vexu-claude` at `/home/vscode/.claude` persists Claude Code settings/secrets.
6. **Knobs:** `shm_size: 2g` for DDS + Gazebo; `cap_add: SYS_PTRACE` for gdb; public DNS (8.8.8.8 / 1.1.1.1) so `rosdep` can reach GitHub from hosts with broken default resolvers.
7. **`novnc` service** (separate container on the shared `x11` network) runs Xvfb + noVNC on port 8080. `DISPLAY` in `vexu` falls back to `novnc:0.0` when the host has none, so Qt/OpenGL apps render into Xvfb and stream to your browser. Linux users with a local display skip it via `docker compose up -d vexu`.

### Per-machine config ([../.env.example](../.env.example))

`.env` sits next to `docker-compose.yml` and is gitignored. Compose auto-loads it. Useful knobs:

- `ROS_LOCALHOST_ONLY=0` — multi-host ROS 2 discovery on the LAN (default is localhost-only).
- `VEXU_COLCON_BUILD_BASE=build-docker` (plus `INSTALL`/`LOG`) — redirect container colcon outputs to separate dirs if you also do native builds on the same checkout.
