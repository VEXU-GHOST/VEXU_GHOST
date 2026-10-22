# Repository guide for coding agents

## 1. Project overview

This is a VEXU/VEXAI competition robotics workspace. The current hardware bringup targets the Push Back robots **PINKY** and **INKY**, with differential-drive control, joystick operation, behavior-tree autonomous routines, LiDAR localization, external color/distance sensing, and inter-robot communication. Camera perception and Nav2 integration exist, but presence in the source does not imply a working or enabled end-to-end robot.

The software spans three runtimes:

- **Linux coprocessor:** ROS 2 Humble, primarily C++17 with Python launch/perception/IO code; Ubuntu 22.04 is the documented platform. Jetson hardware is explicitly supported by scripts.
- **VEX V5 brain:** PROS C++ firmware, sharing selected libraries with Linux through generated symlinks and compile-time device flags.
- **RP2040 sensor host:** Pico SDK firmware exposing generic I2C operations over USB CDC, with sensor-specific initialization and interpretation on Linux.

Simulation uses Gazebo Classic `gazebo_ros` plugins, Xacro/URDF, and RViz. Other major dependencies include Eigen, yaml-cpp, pluginlib, BehaviorTree.CPP/ROS2, robot_localization, Nav2, CasADi/IPOPT, and camera inference dependencies in `push_back_cv`. Inspect each package's `CMakeLists.txt` and `package.xml` before changing dependencies.

## 2. Repository structure

| Path | Responsibility / where to make changes |
| --- | --- |
| `01_Libraries/ghost_util` | Angles, units, math, byte/YAML utilities and graph search |
| `01_Libraries/ghost_control` | PID and motor control; shared motor code also runs on V5 |
| `01_Libraries/ghost_estimation` | Filters, vector maps, particle filter; `src/shared` is a submodule |
| `01_Libraries/ghost_planners` | Trajectories, Bézier/boomerang code and optimization examples |
| `01_Libraries/ghost_serial` | COBS, message parsing, Linux/V5 serial base interfaces |
| `01_Libraries/ghost_v5_interfaces` | Device configuration/data types, hardware serialization, generated PROS header, peer packet codec |
| `02_V5/ghost_pros` | PROS entry point, motor interfaces, serial transport, screen code and embedded toolchain files |
| `02_V5/ghost_sensor_host` | Standalone Pico firmware, protocol notes and I2C debug tool; excluded from colcon |
| `03_ROS/ghost_msgs` | Shared ROS messages and services |
| `03_ROS/ghost_ros_interfaces` | V5/Pico serial nodes, message conversion, competition plugin base, peer communication, bag/timer services |
| `03_ROS/ghost_sensing` | IMU filtering and ball color classification |
| `03_ROS/ghost_localization` | Particle-filter/EKF nodes, map files, localization launch files |
| `03_ROS/ghost_motion_planner_core` | Exported motion-planner library; not a standalone planner executable |
| `03_ROS/ghost_io`, `ghost_io_py`, `ghost_viz`, `ghost_autonomy` | GPIO, speech/music, visualization and auxiliary autonomy code; inspect build targets to establish what runs |
| `04_Sim/ghost_sim`, `ghost_sim_examples` | Gazebo plugins, test/spoof publishers, worlds, model/launch examples and bag playback |
| `10_Examples` | Example ROS packages, robot plugin and control programs |
| `11_Robots/ghost_tank` | Shared tank model/odometry, Pinky/Inky plugins, BT node implementations, XML routines and tests |
| `11_Robots/ghost_push_back` | Current hardware launches, shared/per-robot YAML, URDF and RViz configurations |
| `11_Robots/push_back_cv` | YOLO/depth detection, block tracking, goal interpretation, custom messages and model assets |
| `09_External`, `.gitmodules` | External dependencies; avoid incidental edits to third-party sources |
| `scripts`, `cmake`, `robots.yaml` | Provisioning, build/test/deployment helpers, common compiler settings, V5 configuration selection |

Treat `build/`, `install/`, `log/`, generated `02_V5/ghost_pros/include/robot_config.hpp`, and PROS library symlink trees as outputs. Edit their authoritative sources instead.

## 3. Architecture and data flow

### Hardware launch and competition control

`11_Robots/ghost_push_back/launch/hardware.launch.py` selects PINKY/INKY using `robot_name` and includes the corresponding launch. It starts robot_state_publisher, map server, Nav2 planner/controller lifecycle nodes, RPLIDAR, bag recorder and peer communication. Per-robot launches start V5 serial, competition plugin, IMU filtering, two robot_localization EKFs and `ekf_pf_node`. Pico serial and one classifier per configured COLOR device are added by the parent launch.

RealSense, the CV pipeline, speech/music and GPIO nodes are defined in launch code but commented out of the relevant launch action lists. Invalid robot names omit robot-specific nodes while common nodes still launch.

The control path is:

1. V5 serializes hardware state; `JetsonV5SerialNode` reads it into its own `RobotHardwareInterface`, then publishes `v5/sensor_update` with `SensorDataQoS`.
2. `V5RobotBase::sensorUpdateCallback` receives `/v5/sensor_update`, updates competition state, copies sensor fields into a separate plugin-owned hardware interface, calls `onNewSensorData()`, then dispatches `disabled()`, `autonomous()` or `teleop()`.
3. The callback serializes that plugin interface to `/v5/actuator_command`. The serial node converts and writes commands back to V5. These interfaces are separate copies connected by ROS messages, not one shared object.
4. `competition_state_machine_node` loads `ghost_tank::PinkyPlugin` or `ghost_tank::InkyPlugin` through pluginlib, configures it and spins its ROS node. `TankRobotPlugin` supplies common odometry, model, control and BT behavior; robot subclasses specialize mechanisms/operator behavior.
5. `TankTree` registers XML node names in `src/tank_tree.cpp`, creates a tree from a file and ticks it once per call. Nodes obtain shared model, trajectory, ROS node and other state through blackboard keys. Keep key names/types and XML port contracts aligned.

This ROS control path is driven by incoming sensor callbacks, not an independent guaranteed-frequency command timer. Exceptions in the base mode dispatch are logged, but the callback still publishes the interface's command state; do not assume an exception clears actuators.

### Estimation, navigation and perception

- Tank encoder odometry publishes `/sensors/wheel_odom`. Filtered IMU input is `/estimation/imu/filtered`; the odom and map EKFs consume these according to `ghost_push_back/config/localization_config.yaml`.
- `ekf_pf_node` connects LiDAR/vector-map localization with odometry. Inspect this implementation and the selected YAML before modifying reset, update gating, covariance or frame behavior; alternative `pf_ekf_node` code also exists.
- Robot sensor transforms come from `ghost_push_back/urdf/ghost_push_back.urdf.xacro`; the particle filter obtains the LiDAR offset from TF.
- `GeneratePlannerPath` uses Nav2's `compute_path_to_pose` action. The hardware launch remaps controller output to `/nav2/cmd_vel`, consumed by the `FollowPathControllerServer` BT integration. Merely launching Nav2 does not establish that a selected autonomous XML uses it.
- Optional camera chain: RealSense color/aligned depth/camera info → `cv_detector_array.py` → `/cv/detections` → `block_map_node.py` → `/field/blocks` → `goal_reader_node` → `/field/goals`. Camera launch is separate from the reusable CV launch. Detector code uses TF for map-frame projection.

### Physical communication

- V5 packets use `ghost_serial` framing/COBS and a configuration-dependent binary layout in `ghost_v5_interfaces`; ROS conversion is in `ghost_ros_interfaces/msg_helpers`. Linux runtime hardware YAML must match the config compiled into V5.
- Peer data flows through `/comms/self` → V5 serial → VEXlink → peer V5 → `/comms/other_robot`; `inter_robot_comms_node` quantizes/dequantizes pose/status and publishes the peer TF `map` → `other_robot/base_link`. V5 relays the payload as bytes. Update codec and middleware tests together when changing this contract.
- Pico uses a separate protocol: COBS(`[cmd][len LE16][payload][checksum]`) plus zero delimiter. See both `ghost_ros_interfaces/sensor_host/protocol.hpp` and firmware `src/comms.cpp`; do not substitute V5 framing. Firmware supports eight buses and sixteen recurring reads, with limits of 32 pre-write, 16 post-write and 64 read bytes. Sensor initialization/decoding lives in `jetson_sensor_host_serial_node.cpp`; COLOR and DISTANCE are implemented, IMU/IO_EXPANDER are future work there.
- The ROS serial node owns background serial/reconnect threads and mutex-protected reset operations. Preserve synchronization and shutdown/join behavior when changing transport code.

## 4. Development environment and commands

Run commands at the repository root in Bash. Set the actual checkout path; its spelling here differs from the default `~/VEXU_GHOST` used in scripts.

```bash
export VEXU_HOME="$PWD"
source /opt/ros/humble/setup.bash
# After a successful build:
source "$VEXU_HOME/install/setup.bash"
```

`scripts/setup_env.sh` sources both ROS and the installed workspace, loads robot identity, sets library/plugin paths and optionally restricts Fast DDS to the wired network. On a fresh checkout its install setup file may not exist yet. CMake packages locate `cmake/GhostConfig.cmake` through `VEXU_HOME`.

Provisioning commands present in the repository (inspect before executing; they install/remove system packages and download code):

```bash
git submodule update --init --recursive
./scripts/update_dependencies.sh
./scripts/setup_submodules.sh
```

`update_dependencies.sh` installs ROS/system/Python dependencies, Pico SDK/picotool 2.2.0 and ARM tools. `setup_submodules.sh` also installs architecture-specific Debian packages for optimization, plotting, LiDAR and BT dependencies; it is much more than a submodule fetch. PROS CLI is additionally required for V5 builds.

For targeted Linux development, use direct colcon commands to avoid hardware side effects:

```bash
colcon build --symlink-install --packages-up-to ghost_ros_interfaces --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source "$VEXU_HOME/install/setup.bash"
```

Replace the package with the affected package. `./scripts/ghost build <pkg...>` also selects packages and dependencies, but first attempts `ghost stop`. A full `./scripts/ghost build` skips a list of external packages, omits simulation/viz on aarch64, then **uploads V5 firmware if PROS exists and attempts Pico flashing if its toolchain exists**. Do not use it as a harmless validation command.

Embedded build-only commands, after the ROS header generator has been built:

```bash
bash scripts/build/pros_build.sh
bash scripts/build/sensor_host.sh --no-flash
```

The PROS helper rebuilds shared-library symlinks, generates `robot_config.hpp` from `robots.yaml`, cleans and calls `pros make`. Identity comes from `scripts/robot_name.sh` / `~/robot_name`, with `default_robot` as generator fallback. Inspect the selected mapping before any build/upload. `pros_upload.sh` uploads; `sensor_host.sh` without `--no-flash` flashes.

Hardware launch, on an appropriately configured robot only:

```bash
ros2 launch ghost_push_back hardware.launch.py robot_name:=PINKY
```

`./scripts/ghost run` additionally stops processes and invokes a launcher hardcoded to `/home/ghost/VEXU_GHOST`. `run --v5` runs the brain program first. `set-robot-name`, `configure-os`, `install`, `start`, `stop`, `kill`, `shutdown` and `log` manage identity, OS settings, systemd and power. `install` enables services immediately; stop/kill use broad process-name matching. These are operational actions, not routine checks. See [README.md](README.md) and [scripts/hardware](scripts/hardware).

Simulation entry points exist as `./scripts/launch_sim.sh` and `ros2 launch ghost_sim_examples start_sim.launch.py`. **The latter currently comments out its Gazebo launch action**, while retaining spawning/RViz. Inspect and resolve this before expecting a self-contained simulation. Other launch files in `04_Sim/ghost_sim/launch` are debugging/playback fixtures; establish their prerequisites from source.

## 5. Coding conventions

- Follow the neighboring file: C++ commonly uses two-space indentation, package namespaces, PascalCase classes, camelCase methods, `.hpp` public headers in `include/<package>` and `.cpp` in `src`. Members mix `m_` and trailing `_`; do not rename broadly to normalize them.
- Common CMake defaults are C99/C++17 with `GHOST_DEVICE`, `GHOST_JETSON` and `GHOST_V5_BRAIN` flags. Preserve Linux/PROS portability in shared libraries. The common build-type string is literally `RelWDebInfo`; do not assume standard debug flags are effective without checking compiler output.
- Add dependencies to both CMake and `package.xml`, export/install new interfaces/targets consistently, and preserve existing pluginlib names. New BT libraries need CMake registration as well as factory registration and matching XML ports.
- Use declared ROS parameters for node settings. Device maps are parsed from dedicated YAML paths, not passed as arbitrary nested ROS parameter maps. Keep shared ROS, per-robot ROS, hardware, sensor-host, localization/Nav2 and match-init configuration responsibilities distinct.
- Match local error handling: validation/serialization utilities may throw, ROS nodes use `RCLCPP_*`, V5 reports through its screen queue. Never print debug text on the Pico USB protocol stream.
- Root `uncrustify.cfg` provides formatting settings. Avoid wholesale formatting changes; preserve license notices in existing files.

## 6. Robotics engineering constraints

Verified constraints:

- Tank odometry uses meters/radians internally, positive heading from `(right_distance-left_distance)/wheelbase`, and wraps heading through `WrapAngle2PI`. Wheel dimensions enter some YAML interfaces in inches; BT ports explicitly use tiles/degrees. Convert at boundaries using `ghost_util/unit_conversion_utils.hpp`, and inspect each node's conversions rather than assuming a global unit system.
- Localization uses `map`, `odom`, `base_link` and `lidar_link`. Config comments identify robot X forward/Y left and a 144-inch (3.6576 m) square field with corner-origin map. Preserve consistency among URDF, vector map, occupancy map, BT mirroring and reset poses; do not infer compatibility from filenames alone.
- Normal V5 loops use `loop_frequency = 10` **milliseconds**, despite the name. The reader waits half that period. A separate timeout task zeros motor control modes/commands after `cmd_timeout_ms = 50`; disabled handling also zeros motors. These are motor protections, not proof every pneumatic/output is reset.
- V5 actuator updates use `actuator_update_lock`. Preserve lock coverage and avoid blocking operations in loops. ROS serial reconnect logic uses a 100 ms incoming-message timeout. These are software timing choices, not hard real-time guarantees.
- Pico port numbers are zero-based (physical input minus one), with hardware I2C on ports 0/1 and PIO I2C on 2–7. Address remapping and per-sensor read intervals are configured on ROS. Distance messages include explicit `distance_mm`; do not treat this as meters.
- Hardware bringup resolves V5/Pico by `/dev/serial/by-id` type patterns, falling back to `/dev/ttyACM1` and `/dev/ttyACM0`. LiDAR is explicitly `/dev/ttyUSB0`, 256000 baud. Recheck actual connected devices before debugging port errors.

Recommended precautions for changes affecting motion: inspect current-limit/control-mode/voltage behavior on both processors; preserve timeout and disabled paths; keep the chassis restrained or wheels clear for initial tests; have a reliable disable mechanism; verify sensor direction, encoder scale and mechanism outputs before field motion. Do not retune gains, limits, hardware ports or frame conventions without explicit rationale and hardware validation. Simulation success does not establish hardware safety.

## 7. Testing and validation

Tests are registered with `ament_add_gtest` across utility, serial, control, device-interface, planner, ROS-interface and tank packages. Examples cover COBS/parsers, PID/current limiting, config loading/serialization, ROS conversions, peer codec/TF loopback, sensor-host protocol, tank odometry and trajectories. Some `test/` programs are visualization/debug executables rather than automated tests; check CMake registration.

After building the affected package and sourcing the overlay:

```bash
colcon test --packages-select ghost_ros_interfaces --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

Select all affected packages, including downstream consumers of changed public/wire interfaces. `bash scripts/test/test_full.sh` runs the workspace suite with explicit exclusions including `ghost_sim`; `bash scripts/test/test_pkg.sh <pkg...>` builds dependencies and tests selected packages but does not itself request a failing exit code for failed tests. Inspect results explicitly.

`test_sensor_host_protocol` is hardware-free; `test_sensor_host_hardware` skips when the board cannot be found/opened. A skipped integration test does not verify hardware. Peer tests require working ROS middleware even without physical robots. Gazebo tests are not evidence of Push Back hardware coverage.

For hardware-dependent changes, additionally validate matching Linux/V5 configs, binary packet sizes, reconnect/lost-command behavior, disabled transitions, sign/scale of sensor data and bounded actuator response. Check TF and estimates in RViz before enabling autonomous motion. For camera changes, validate calibrated depth/TF and inference on the target GPU.

When tools, dependencies, tests or hardware are unavailable, record the exact unexecuted checks and blockers. Do not silently skip them or report old `build/`/`log/` results as a new pass. This guide was produced by source/script inspection; no full build, ROS tests or hardware execution was performed for the documentation change.

## 8. Operational rules for AI coding agents

1. Read the affected implementation, its configuration, callers and CMake/test registrations before editing. Use active launch action lists to establish what runs.
2. Prefer targeted changes and preserve public APIs, ROS topic/QoS contracts, blackboard keys and binary layouts unless the task requires a coordinated change.
3. Do not introduce dependencies without explaining why existing code cannot serve the requirement. Shared-library edits must consider both Linux and PROS consumers.
4. Do not edit generated PROS headers or symlink copies as authoritative source. Do not overwrite unrelated work; inspect `git status` before and after changes.
5. Do not run provisioning, flashing, broad process kills, service activation or shutdown as routine validation. Inspect script side effects and remain within the user's authorized scope.
6. Update relevant tests/configuration/docs with behavior changes; run meaningful checks and report commands, results, assumptions and unresolved risks accurately.
7. Never infer safety from simulation, a successful compilation, caught exceptions or a hardware test that skipped.

## 9. Common development workflows

### Add or modify a robot/BT subsystem

Trace per-robot launch → pluginlib type → `TankRobotPlugin`/Pinky/Inky implementation → model/hardware interface. For a new BT node, add its header/source, CMake target/link entry and `TankTree` factory registration; define ports and blackboard dependencies explicitly, then update the intended XML. Verify the chosen `*_init_config.yaml` resolves to a real XML under the installed `ghost_tank/config` directory. Exercise failure, halt and competition transition behavior as well as success.

### Integrate a sensor or actuator

For V5 devices, update the selected hardware YAML and corresponding device/config serializers and ROS conversions as needed, run device tests, regenerate firmware and deploy matching Linux/V5 versions. For Pico devices, update the per-robot `*_sensor_host_config.yaml`; add sensor-specific init/decoding in the ROS driver if the type is unsupported. Check firmware transaction limits and protocol tests before deciding a firmware change is necessary. Update URDF sensor transforms and consumers together when geometry changes.

### Modify control, odometry or navigation

Start with `tank_model.cpp`, `tank_odom.cpp`, robot plugin callbacks, affected BT node and current robot YAML. Verify units, gear ratio, encoder ticks, left/right sign, timestep and gain assumptions. For Nav2, inspect `nav2_config.yaml`, planner action construction and controller output relay. Add numerical tests for changed math and validate timeout/reset/disable behavior before physical trials.

### Change a communication interface

Trace device data → hardware serialization → transport → ROS conversion → consumer in both directions. Update schema, codec, firmware and consumers together; test round trips, corrupt/truncated inputs and size limits. Peer packet changes additionally require codec and middleware integration tests. Pico and V5 protocols are distinct.

### Diagnose build/runtime failures

Check `VEXU_HOME`, ROS/overlay sourcing, package discovery and required external dependencies first. Use package-specific colcon builds and `build/<package>/compile_commands.json` rather than rebuilding/flashing everything. Inspect `log/` for current failures. For runtime failures, check actual launch actions/parameters, serial device permissions (`dialout` per README), robot identity, plugin exports, BT paths, TF and topic QoS. `ghost log` exposes systemd logs on configured robots; bag recorder services and RViz configurations support runtime inspection. Avoid broad cleanup: `ghost clean` deletes build/install/log artifacts.

## 10. Known limitations and caution areas

These are source observations, not a claim that hardware was exercised:

- `02_V5/ghost_pros/src/main.cpp` defines `AUTON_SELECTOR_SCREEN_TEST 1`; `initialize()` constructs the selector and loops indefinitely instead of executing normal hardware/serial initialization. The normal loop/watchdog description above applies to the alternate branch. Do not assume a newly built image runs the normal stack.
- Hardware launch/config files contain `~/VEXU_GHOST` and `/home/ghost/VEXU_GHOST` paths; `VEXU_HOME` alone does not relocate every runtime asset. `ghost_push_back/CMakeLists.txt` installs config/launch/RViz but not its URDF directory, while hardware launch reads that URDF directly from the source checkout.
- `robots.yaml` retains entries pointing into absent `11_Robots/ghost_over_under`; its `GHOST_TANK` config path is also absent in this checkout. Verify each selected mapping instead of assuming all listed robots are usable.
- A source file's existence does not guarantee it has a build/test target; consult package CMake registrations before treating debug/visualization programs as automated coverage.
- `.github/workflows/workflows.yml` calls nonexistent `scripts/build.sh -r` and `scripts/test_full.sh`; actual helpers are in `scripts/build/` and `scripts/test/`. CI configuration is not proof the current checkout builds.
- Submodule state needs repair/verification: inspection found `ghost_estimation/src/shared` uninitialized and `git submodule status` failed for `09_External/pico-sdk` because no mapping exists in `.gitmodules`. Do not automatically rewrite submodule metadata as part of an unrelated task.
- `ghost_sim_examples/start_sim.launch.py` leaves simulator startup commented out. Simulation fixtures and older robot assets need individual verification; no complete Push Back simulation/hardware acceptance procedure is established here.
- Sensor-host `PROTOCOL.md` labels itself DRAFT and contains proposed/future details. Firmware and ROS codec are authoritative for supported limits/features. The host is a generic bridge; comments mentioning older sensor-specific firmware files are not proof those files exist.
- CV TensorRT `.engine` files depend on the target GPU/TensorRT version; detector code documents rebuilding locally and fallback to `.pt`. Do not assume committed engines or Python/GPU dependencies work on another machine. Default hardware bringup disables the camera/CV pipeline.
- Competition plugin loading catches errors but subsequently dereferences the plugin pointer; error logging is not a robust recovery guarantee. ROS mode exceptions likewise do not automatically clear previous commands. Treat these paths cautiously.

Useful repository improvements, when separately requested: repair stale CI/config references and submodule metadata, make runtime paths relocatable, document a reproducible simulator bringup, and establish explicit hardware acceptance checks and camera dependency versions.
