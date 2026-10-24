# Description
This repository contains code intended to support VEXU and VEXAI teams interested in leveraging advanced programming techniques on their competition robots.

# Setup

The recommended flow is Docker — works the same on macOS, Windows, and Linux, and you never have to match a specific Ubuntu version on the host.

**Windows / macOS users:** start at [SetupMyEnvironment.md](SetupMyEnvironment.md) for Docker Desktop + SSH/Git setup, then come back here.


## 2. Clone the repo (on your host, not inside a container)

```bash
git clone git@github.com:VEXU-GHOST/VEXU-GHOST.git
cd VEXU-GHOST
git submodule update --init --recursive
```

SSH keys and `~/.gitconfig` on your host are bind-mounted read-only into the container, so git inside the container uses the same identity.

## 3. Build the image and start the dev container

> [!NOTE]
> Recommended for most users: open this repo in VS Code and use **Dev Containers: Reopen in Container**. VS Code will manage container start/attach for you and auto-install the recommended extensions from [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json). Use the CLI `docker compose ...` flow below if you prefer terminal-only control.


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

Follow: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

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
```

#### Build Repository
```sh
./scripts/ghost build
```

#### Start Simulator
```sh
./scripts/launch_sim.sh
```

### Real robot USB access

```bash
sudo usermod -a -G dialout $USER
```

# The `ghost` command

`scripts/ghost` is the single entry point for building the code and operating the robot. Most subcommands only work once the machine has been provisioned as a robot (a name written to `$VEXU_HOME/robotname`); off-robot only `run`, `build`, `clean`, and `set-robot-name` are available.

## Initial Jetson setup (before the `ghost` symlink exists)

On a fresh Jetson the `ghost` shortcut isn't installed yet, so call the script by its path. From `~/VEXU_GHOST`:

```sh
./scripts/ghost set-robot-name clyde   # name this robot (writes $VEXU_HOME/robotname)
./scripts/ghost configure-os           # one-time OS setup; also runs install and creates the `ghost` symlink
```

After `configure-os`, the `ghost` command is available system-wide (symlinked into `/usr/local/bin`), so you can drop the `./scripts/ghost` prefix and just run `ghost <command>`.

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
- `ghost set-robot-name <name>` — provisions this machine as a robot by writing its name to `$VEXU_HOME/robotname` (git-ignored, so it stays local to the machine). If unset, `example_robot` is used and a red warning is logged.

# Match bag recording

With `should_record: true`, a bag is recorded for autonomous (it stops when
autonomous ends) and a second bag for driver control (it starts when driver
control begins after an autonomous period and stops on the
driver-control-to-disabled transition). The break between them is not recorded,
and driver control without a preceding autonomous period is not recorded. Bags are saved in
`~/bags/<robot>_<MM_DD_YYYY>_<number>_1` for autonomous and `..._<number>_2` for driver
control (for example `blinky_10_23_2026_3_1` and `blinky_10_23_2026_3_2`). The number is
the next unused recording number for that robot and day, shared by both halves.
Manual recordings (bag button, `start_bag_recorder_client`) are saved as
`~/bags/<robot>_<MM_DD_YYYY>_<number>`.

No bag or match video is started when the Jetson has less than 20 GB free
(`min_free_disk_gb_to_start` for bags, `min_free_disk_gb` for the camera node).
A recording that is already running is not cut off by this check.

Camera topics (`/cv/*` and `/camera/*`) are not recorded in bags. Match video is saved
separately by `cv_camera_node` in `~/match_videos`.

The recorder writes all other topics it normally discovers to one uncompressed MCAP (`fastwrite` preset)
file during the match. After stop, ROS 2 closes that file and compresses it
losslessly with Zstd using one worker (`--compression-mode file
--compression-format zstd --compression-threads 1`). File splitting is disabled
so compression does not start during a match. This preserves recorded messages
and their timestamps without reducing sensor resolution or sample rates.

On stop, recorder threads are lowered to CPU nice level 10 for background
finalization. Another match can start recording at normal priority while an
older bag finishes. The bag LED turns off when recording stops; compression may
still be running. Wait for the completion log before powering off or copying the
bag. Allow free space for both the original and compressed file during this step.
ROS removes the original after successful compression; the final bag contains
`metadata.yaml` and a `.mcap.zstd` file. If compression fails, inspect recorder
logs and keep any remaining original file.

Install the runtime dependencies on the Jetson and playback computer (also
declared in `ghost_ros_interfaces/package.xml`):

```sh
sudo apt install ros-humble-ros2bag ros-humble-rosbag2-transport \
  ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-storage-mcap \
  ros-humble-rosbag2-compression-zstd
```

After rebuilding `ghost_ros_interfaces` and starting the updated robot stack,
new matches use this behavior; existing bags are not changed. Replay a completed
bag using `ros2 bag play ~/bags/<bag-directory>` and view the replayed topics in
RViz. PlotJuggler can open the bag's `metadata.yaml` with its ROS 2 bag loader
when the compression plugin and custom message packages are available.
File-level compression requires decompressing the file for playback, so allow
space and startup time on the visualization computer.

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
