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

`scripts/ghost` is the single entry point for building the code and operating the robot. Most subcommands only work once the machine has been provisioned as a robot (a name written to `/etc/ghost/robot_name`); off-robot only `run`, `build`, `clean`, and `set-robot-name` are available.

## Initial Jetson setup (before the `ghost` symlink exists)

On a fresh Jetson the `ghost` shortcut isn't installed yet, so call the script by its path. From `~/VEXU_GHOST`:

```sh
./scripts/ghost set-robot-name clyde   # name this robot (writes /etc/ghost/robot_name)
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

## Foxglove connection

The competition hardware launch (`ghost_override`, used by both Clyde and Blinky) and the
simulation launch (`ghost_sim_examples`) each start one Foxglove Bridge on port 8765. Do not
start another bridge on that port alongside them. Native setup installs `ros-humble-foxglove-bridge`
through `scripts/update_dependencies.sh`; the Docker image installs it with apt as well. Both
launch packages also declare it as a runtime dependency for rosdep.

Print the connection address with the repository CLI:

```bash
ghost foxglove-url            # ws://orinx.local:8765
ghost foxglove-url localhost  # ws://localhost:8765 for local simulation
```

Off-robot, run `./scripts/ghost foxglove-url` directly. In Foxglove, select Open connection,
choose Foxglove WebSocket, and paste the URL. The command only prints an address; it does not
start the bridge or test connectivity. `orinx.local` must resolve to the robot computer, and your
laptop must be able to reach its port 8765. A different hostname or IPv4 address can be passed as
the optional host argument.

For local simulation:

```bash
./scripts/launch_sim.sh
# Headless alternative for diagnostics:
./scripts/launch_sim.sh sim_gui:=false rviz:=false
```

Docker publishes port 8765 to the host; recreate the dev container after changing Compose
configuration, and rebuild its image after dependency changes. ROS runs inside the container,
while Foxglove can run on the host.

A 3D panel can use `/robot_description`, `/tf`, and `/tf_static`; sensor panels require the
corresponding sensor topics to be published.

## Teleop drive style

Clyde and Blinky default to tank drive (left stick Y = left side, right stick Y = right side).
To switch to arcade drive (left stick Y = forward/back, right stick X = turn), set
`arcade_drive: true` under `tank_robot_plugin` in
`11_Robots/ghost_override/config/base_ros_config.yaml` (or a per-robot ros_config), then rebuild
and relaunch.

# Automatic on-device recording

The existing C++ `bag_recorder_service` starts recording when the robot's
`ghost_override` hardware launch runs. There is no separate Python supervisor
or recording shell script. It inherits the robot's Humble/custom-message/DDS
environment from `scripts/setup_env.sh`.

Recordings are stored in `~/bags` (`/home/ghost/bags` on the Jetson). Every session
has a UTC timestamp, process ID, and random suffix. It contains SQLite bag files
and `metadata.yaml`; copy the entire session directory for later replay using
the matching Ghost message definitions. Existing recordings are never deleted
automatically. New bag files are split at approximately 1 GiB or five minutes.
All discovered topics are recorded, including publishers that start later.
Camera topics, when enabled, may consume substantial storage.

Configure `bag_recorder_service.ros__parameters` in
`11_Robots/ghost_override/config/base_ros_config.yaml`:

- `autostart: true`: request recording automatically on startup.
- `enable_button_control: false`: ignore physical button messages during
  unattended recording. Set true to restore the existing button behavior.
- `bag_directory`: optional absolute storage path; defaults to `$HOME/bags`.
  The directory is private to the robot's account and administrators.
- `disk_space_required_kb: 5242880`: stop below 5 GiB free on the recording disk.
- `disk_resume_margin_kb: 1048576`: resume after free space reaches 6 GiB.
- `disk_check_period_s: 5`: check the recorder and free space every five seconds.
- `max_bag_size_bytes: 1073741824` and `max_bag_duration_s: 300`: file splitting.

A failed recording process is reaped and retried after ten seconds, rounded up
to the next check. One recorder node owns a directory at a time. Start/stop
services remain available: a stop request pauses recording until another start
request or node restart. Keep the existing competition `should_record: false`
setting for uninterrupted logging, because a deliberate stop service call is
still honored. The existing bag LED is updated while recording is active;
it indicates a running recorder process, not verified message delivery.

Low-space recording resumes automatically after data is offloaded. The reserve
is sampled, not a hard quota; data arriving faster than the reserve allows can
still fill the disk between checks. Sudden power loss can damage the active bag.

After copying the updated source to the Jetson, build the affected packages:

```bash
cd ~/VEXU_GHOST
source scripts/setup_env.sh
colcon build --symlink-install --packages-select ghost_ros_interfaces ghost_override
source install/setup.bash
```

Refresh the existing robot user service on the Jetson:

```bash
./scripts/ghost install
systemctl --user daemon-reload
systemctl --user restart ghost-hardware.service
```

The existing autologin setup starts user services. To start without a login
session, enable lingering once with `sudo loginctl enable-linger ghost`.
The hardware wrapper no longer depends on `/dev/tty`, and service shutdown allows
time for the recorder to finish. `ghost restart`, `ghost stop`, and
`ghost shutdown` use the graceful service path. `ghost kill` is still an emergency
force-stop and can damage the current recording. Inspect logs with
`journalctl --user -u ghost-hardware.service`.
