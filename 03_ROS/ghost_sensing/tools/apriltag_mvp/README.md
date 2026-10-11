# ROS 2 AprilTag MVP on Windows with WSL 2

This MVP runs the **upstream `apriltag_ros` detector on ROS 2 Humble**, using the same ROS nodes intended for the Jetson:

```text
D435 -> realsense2_camera -> image_proc/rectify_node -> apriltag_ros/apriltag_node
                                                        |-- /apriltag/detections
                                                        `-- camera-to-tag TF
```

The launch and configuration live in [`launch/apriltag.launch.py`](../../launch/apriltag.launch.py), [`config/apriltag.yaml`](../../config/apriltag.yaml), and [`config/apriltag_camera.yaml`](../../config/apriltag_camera.yaml). This replaces the previous Python/OpenCV detector. Upstream packages use OpenCV internally for image processing and pose estimation; we do not maintain a separate detector.

This estimates tags relative to the camera, not the robot's field position. It does not start robot control or feed the EKF. The D435 uses RGB only here; it has no IMU. Camera mounting TF and surveyed field landmarks are needed later for robot localization.

## Scope and deployment handoff

This is an independently launched **local MVP**. It runs Linux ROS processes on the Windows computer through WSL 2; it is not a native Windows ROS installation. It approximates the Jetson's Ubuntu/Humble software interfaces, not its ARM CPU, JetPack drivers, USB behavior, or timing. No hardware launch or robot service is changed to start this detector.

The deployment team can reuse these reviewable artifacts:

| Artifact | Purpose |
| --- | --- |
| [`packages-core.txt`](packages-core.txt) | ROS base, actual AprilTag node, image rectification, TF tools, and bag replay. |
| [`packages-camera.txt`](packages-camera.txt) | Optional RealSense ROS packages and USB inspection tool. |
| [`setup_dependencies.sh`](setup_dependencies.sh) | Installs/checks those manifests on Ubuntu 22.04 and enforces detector version >= 3.4.0. Defaults to a read-only check. |
| [`apriltag.yaml`](../../config/apriltag.yaml) and [`apriltag.launch.py`](../../launch/apriltag.launch.py) | Shared detector settings and ROS wiring. |
| `--versions` output and recorded bags | Evidence of exactly which selected packages and observations were tested. |

The installer only installs Linux packages when explicitly run with `--install`. It does not configure ROS apt trust/sources, modify shell startup, provision a Jetson, build the robot workspace, create services, or start nodes. The ROS apt repository is a one-time prerequisite; follow the official setup below. Its normal apt confirmation shows the proposed package transaction.

These manifests define a repeatable installation procedure, **not an exact dependency lock**: apt selects available candidates. Save the version report after a successful test. For deployment, qualify the same upstream detector/wrapper versions on the target architecture and preserve a reviewed apt snapshot or image if exact reproducibility is required. The report lists selected packages, not every transitive dependency. ARM64 and AMD64 Debian build versions can differ; do not copy Windows/WSL binaries to the Jetson. Camera driver/backend qualification is a separate deployment step.

## Accessing this checkout from WSL

A WSL distribution belongs to the computer, not to a particular terminal. From PowerShell, enter Ubuntu 22.04 at the existing Windows checkout:

```powershell
wsl -d Ubuntu-22.04 --cd /mnt/c/Users/manje/GHOST-VEX/VEXU-GHOST
```

In that Ubuntu terminal:

```bash
pwd
ls 03_ROS/ghost_sensing/launch/apriltag.launch.py
```

This is the same directory as `C:\Users\manje\GHOST-VEX\VEXU-GHOST`; no clone/copy inside WSL is required. Teammates substitute their own Windows checkout path. Files are shared, but Linux packages, processes, and environment variables belong to WSL. Source ROS separately in every Linux terminal. PowerShell's Python environment does not provide ROS to WSL.

Using `/mnt/c` is sufficient for these launch/configuration files. For large recordings or full Linux builds, prefer the WSL filesystem for performance; avoid sharing generated build/install directories across operating systems. See [Microsoft's filesystem guidance](https://learn.microsoft.com/en-us/windows/wsl/filesystems).

## 1. Use Ubuntu 22.04 in WSL 2

The repository targets Humble on Ubuntu 22.04. In PowerShell:

```powershell
wsl --list --verbose
wsl -d Ubuntu-22.04
```

If that distribution is absent, install it with `wsl --install -d Ubuntu-22.04`, finish its first-run setup, and check that its WSL version is 2. Avoid using a newer default Ubuntu distribution for Humble. On the reviewed workstation, Ubuntu 22.04 is already present, while the distribution named simply `Ubuntu` is 26.04.

In Ubuntu 22.04, complete the locale, apt-source, and system-prerequisite sections of the [official Humble Debian-package instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). The script below installs `ros-humble-ros-base` along with the detector, so a full desktop installation is optional. The ROS apt repository must be configured before installation; otherwise the script reports missing apt candidates and stops.

From the repository root in Ubuntu:

```bash
bash 03_ROS/ghost_sensing/tools/apriltag_mvp/setup_dependencies.sh --check --camera
bash 03_ROS/ghost_sensing/tools/apriltag_mvp/setup_dependencies.sh --install --camera
source /opt/ros/humble/setup.bash
ros2 pkg executables apriltag_ros
ros2 pkg executables image_proc
```

The first check is expected to report missing packages on a fresh machine. `--install` updates apt metadata, checks availability/version requirements, installs the selected packages, and checks installed versions. Re-running it is supported. Omit `--camera` on machines used only for recorded-bag detection. Neither mode installs a kernel module or resolves WSL USB forwarding.

Expect `apriltag_node` and `rectify_node` in the executable lists. The supplied detector YAML targets **apriltag_ros 3.4.0 or newer**, released for Humble. No pip environment or cloned detector source is required. Save the selected package inventory with test results:

```bash
mkdir -p 03_ROS/ghost_sensing/tools/apriltag_mvp/output
bash 03_ROS/ghost_sensing/tools/apriltag_mvp/setup_dependencies.sh --versions --camera \
  > 03_ROS/ghost_sensing/tools/apriltag_mvp/output/installed-packages.tsv
```

Only use a report from a successful (exit code 0) command; a missing package produces a partial report and a nonzero exit. Also record the Git commit (`git rev-parse HEAD`), camera firmware, tag family/size, and launch arguments. Optional image visualization can be installed separately with `sudo apt install ros-humble-rqt-image-view`.

Source `/opt/ros/humble/setup.bash` in every ROS terminal. Running directly in this WSL distribution avoids adding Docker USB/device mapping to the first test. The existing Docker setup remains usable for development and replay, but is not automatically configured for live D435 access.

## 2. Forward the D435 USB device into WSL

Microsoft documents USB forwarding through [usbipd-win](https://learn.microsoft.com/en-us/windows/wsl/connect-usb). Install it from PowerShell:

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
```

Identify the D435's bus ID. In **administrator PowerShell**, share that specific device (replace `BUSID`):

```powershell
usbipd bind --busid BUSID
```

Keep an Ubuntu 22.04 terminal open. In PowerShell, attach it:

```powershell
usbipd attach --wsl --busid BUSID
```

In Ubuntu, verify that `lsusb` lists the camera. Windows applications cannot use it while attached to WSL; close RealSense Viewer before attaching. Attachment may need repeating after reconnecting or restarting WSL. Return it to Windows with `usbipd detach --busid BUSID`.

**USB enumeration is only the first check.** WSL camera streaming depends on its kernel, USB forwarding, permissions, and the installed librealsense backend. Seeing the D435 in `lsusb` does not guarantee the driver can stream. If the node reports no device, inspect `/dev/video*`, permissions, and driver logs. The SDK's user-space RSUSB backend is a possible fallback when V4L2 support is missing; it requires a compatible SDK/wrapper build, not just a launch argument. Do not install the repository's historical Jetson kernel modules in WSL. See [SDK Linux installation/backend guidance](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md).

If forwarding blocks a teammate, bag replay below still runs the real detector without USB access.

### Permission denied opening `/dev/video*`

If the camera log says `Cannot open '/dev/video0' ... Permission denied`, stop the camera node with Ctrl+C and inspect permissions in Ubuntu:

```bash
ls -l /dev/video*
id
```

When the device nodes are owned by `root:video` with mode `crw-rw----`, add your Linux user to that group and refresh the current shell's membership:

```bash
sudo usermod -aG video "$(id -un)"
newgrp video
id
source /opt/ros/humble/setup.bash
```

Confirm `video` appears in `id`, then restart the camera launch. Existing terminals/processes retain their old groups; refresh each terminal that starts the camera. Group membership persists, while a broad `chmod` on device files would not be a durable fix. On the reviewed workstation this missing membership caused access errors on `/dev/video0` through `/dev/video5`; fixing it does not by itself validate streaming or other USB permissions.

## 3. Launch from the checkout without building the robot stack

In Ubuntu, change to the repository root. For the current Windows checkout:

```bash
cd /mnt/c/Users/manje/GHOST-VEX/VEXU-GHOST
source /opt/ros/humble/setup.bash
```

Other teammates should substitute their checkout path. Print a [reference AprilTag](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11), preserving its white margin. Measure the tag's outer black square edge, excluding the white margin, in meters. The example assumes **0.10 m**; replace it with your actual measurement. Incorrect size scales estimated distance incorrectly.

```bash
ros2 launch "$PWD/03_ROS/ghost_sensing/launch/apriltag.launch.py" \
  tag_size:=0.10 family:=36h11
```

This starts the upstream camera executable at `640x480 @ 30 FPS`, rectifies RGB using its calibration, and starts the detector. Image timestamps are preserved. The camera YAML explicitly sets both `color_qos` and `color_info_qos` to `SENSOR_DATA` (best-effort, volatile). The detector also uses sensor-data QoS and the calibration's projection matrix `P` for the rectified image. The camera is launched directly because wrapper 4.58.4's `rs_launch.py` does not expose these QoS settings as ordinary launch arguments.

If another launch already owns the camera, use `start_camera:=false` and ensure that its image and CameraInfo QoS match (see troubleshooting below). The camera package is only looked up when camera startup is requested. `tag_size` is required and must be finite and positive; there is no assumed physical size. `camera_params_file` can override the camera YAML when this launch owns the camera.

To try another camera-supported profile:

```bash
ros2 launch "$PWD/03_ROS/ghost_sensing/launch/apriltag.launch.py" \
  tag_size:=0.10 color_profile:=1280x720x30
```

For custom camera topics, set `start_camera:=false`, `image_topic`, and `camera_info_topic` to absolute topic names. `rectified_topic` can also be overridden. These select processing inputs/outputs; they do not rename the built-in camera node's streams. Use `params_file:=/path/to/settings.yaml` to override detector tuning. The launch arguments override the file's family and default tag size. Per-tag sizes, if added to the YAML, override that default.

## 4. Inspect results

In a second sourced Ubuntu terminal:

```bash
ros2 node list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_rect
ros2 topic echo /apriltag/detections
```

Run the monitoring commands individually; stop each with Ctrl+C. Look for the printed tag's ID, corners, and decision margin. Empty detections arrays mean frames are processed but no tags were found. No messages suggests camera, calibration, timestamp, or QoS issues.

For a visible `36h11` tag with ID 0:

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame 'tag36h11:0'
```

Verify the camera frame from the image header if your driver uses different names. The optical convention is x right, y down, z forward. Compare distance with a tape measure. TF can retain the last observation after a tag disappears: check timestamps and current detections, not just whether a lookup succeeds.

For a live image, run `ros2 run rqt_image_view rqt_image_view` under WSLg and select `/camera/camera/color/image_rect`. This shows the image; upstream `apriltag_ros` does not supply the old custom bounding-box preview. RViz's TF display can show tag frames. Topic/TF checks work without a GUI.

## 5. Share a ROS bag with teammates

### Calibration QoS mismatch

If camera logs report `DURABILITY_QOS_POLICY`, inspect both raw-image and CameraInfo endpoints with `ros2 topic info TOPIC --verbose`. On the tested system, the RealSense default image QoS (`SYSTEM_DEFAULT`) resolved to transient-local, while CameraInfo (`DEFAULT`) was volatile. Humble image_proc 3.0.9 copies the image publisher QoS to both subscriptions, causing its calibration subscription to become incompatible.

The supplied camera YAML fixes this by setting both streams to `SENSOR_DATA`. For an externally launched camera, stop the detector, then apply these settings in a sourced Ubuntu terminal:

```bash
ros2 param set /camera/camera enable_color false
ros2 param set /camera/camera color_qos SENSOR_DATA
ros2 param set /camera/camera color_info_qos SENSOR_DATA
ros2 param set /camera/camera enable_color true
```

Restart the detector afterward so the rectifier re-evaluates publisher QoS. Runtime parameter changes last only until the camera node exits; use the configured MVP launch for subsequent sessions. These commands briefly interrupt RGB capture. An empty `/apriltag/detections` array then confirms processing but not a successful tag identification.

The separate RealSense `clock fit re-based` warning concerns timestamp mapping. It is not fixed by changing QoS. Verify monotonic timestamps and timing stability before treating this MVP as a localization-quality sensor.

### Recording and replay

Record the raw image and its synchronized calibration in a separate terminal:

```bash
ros2 bag record -o /tmp/d435_apriltag_sample \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /tf_static
```

Stop with Ctrl+C. Use a new output directory for each recording and share the complete bag directory. Include tag family, measured size, camera profile, lighting/distance, and package versions.

On another device, stop live camera/detector processes, then run from the repository root:

```bash
ros2 launch "$PWD/03_ROS/ghost_sensing/launch/apriltag.launch.py" \
  tag_size:=0.10 start_camera:=false use_sim_time:=true
```

In another sourced terminal:

```bash
ros2 bag play /path/to/d435_apriltag_sample --clock
```

Use the recorded tag size. Both processing nodes use simulated time, and the launch does not need the RealSense package in this mode. Replay needs image and calibration messages with matching timestamps, not just a JPEG. Do not run live capture alongside replay on the same topics.

## Robot integration later

`ghost_sensing/package.xml` declares the runtime dependencies. The existing CMake install rule installs `launch/` and `config/`. After building the workspace:

```bash
ros2 launch ghost_sensing apriltag.launch.py tag_size:=0.10 start_camera:=false
```

Let hardware startup own the camera when integrating with the robot. Add surveyed-landmark pose conversion and EKF fusion after validating detection and camera-relative pose. See the [ROS overview and roadmap](../../../README.md).

The earlier Python virtual environment and generated images remain ignored local artifacts and are not used by this ROS workflow.

## Validation status

The launch wiring was checked against Humble `image_proc` and `apriltag_ros` 3.4.0. Python syntax, package XML, and relative documentation links passed static checks. After installing Humble and granting the WSL user video-group access, the D435 streamed RGB. A live QoS mismatch was identified and corrected: image_proc 3.0.9 and apriltag_ros 3.4.0 then received matching calibration, and `/apriltag/detections` emitted a timestamped empty array. This validates live frame processing, not successful tag identification or pose accuracy. Persistent camera configuration was added to reproduce the runtime QoS correction. Timestamp stability still needs validation; previous synthetic OpenCV detector tests do not validate this ROS pipeline.

The dependency script passed Bash syntax validation and was run in read-only mode against the Windows-mounted checkout from WSL: Ubuntu 22.04 correctly reported missing ROS packages, help completed successfully, and Ubuntu 26.04 was rejected. The `--install` transaction has not been executed or validated on this workstation. These checks confirm that the repository does not need to be cloned into WSL to use the workflow.

References: [AprilTag ROS node](https://github.com/christianrauch/apriltag_ros), [Humble rectifier](https://github.com/ros-perception/image_pipeline/blob/humble/image_proc/src/rectify.cpp), [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros).
