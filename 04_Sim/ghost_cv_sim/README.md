# ghost_cv_sim — CV model test bench

Stop setting up a physical field to find out if the vision model is any good. This package
puts a 12 ft VEX field in Gazebo, scatters game objects on it, flies a camera around, runs
**your** detector on what the camera sees, and scores it against the simulator's ground truth.

```
 Gazebo ── camera image ──► cv_detector ── detections ──► cv_evaluator ──► live page + report
   ▲                          (your model)                   ▲  ▲
   │ teleport                                                │  └─ scene_manager: where every object is
   └── camera_scanner ── camera pose (per frame) ────────────┘
```

## Run it

Inside the dev container (rebuild the image once so `rosdep` picks up the new package, or just
run `./scripts/build.sh` — the dependencies are all part of ROS desktop):

```bash
./scripts/build.sh
./scripts/launch_cv_sim.sh backend:=color_blob          # smoke test: no key, no weights
```

If your existing image lacks them (a fresh `docker compose build` installs both via `rosdep`):
`apt-get install -y python3-opencv ros-humble-teleop-twist-keyboard` (the second is only for `mode:=manual`).

Then open **http://localhost:8090** on your host (needs `docker compose up -d` after pulling this
change so the port is published). You see the camera frame with ground truth and detections
overlaid, plus live precision / recall / mAP, recall by distance, and a per-class table.

Your Roboflow model (the same hosted workflow the Jetson `cv_camera_node` calls):

```bash
export ROBOFLOW_API_KEY=...        # on the HOST before `docker compose up -d`, or in the container shell
./scripts/launch_cv_sim.sh                             # default backend:=roboflow
```

Overnight-style batch run — 20 random scenes × 10 views, writes a report, exits:

```bash
./scripts/launch_cv_sim.sh mode:=random num_scenes:=20 views_per_scene:=10 seed:=1
```

Reports land in `cv_sim_reports/<timestamp>/` (git-ignored): `summary.txt`, `summary.json`,
`frames.csv`, and `failures/*.jpg` — the frames with misses/false alarms, drawn up, so you can
see *what* the model gets wrong instead of just a number.

## Bring your own model

| `backend:=`   | what it runs |
|---------------|--------------|
| `roboflow`    | hosted Roboflow workflow; `model_id:=...` to change model; key from `$ROBOFLOW_API_KEY` or `/etc/ghost/roboflow_api_key` |
| `ultralytics` | local weights: `weights:=/vexu/best.pt` (`pip3 install ultralytics` in the container first) |
| `custom`      | any Python function: `custom_callable:=my_pkg.my_module:detect` |
| `color_blob`  | HSV colour thresholds. **Not a model** — a pipeline smoke test and a baseline to beat |

A custom detector is just:

```python
def detect(bgr):                      # bgr: HxWx3 uint8 numpy array
    return [{"class_name": "RedBluePin", "confidence": 0.93,
             "x": 320, "y": 240, "width": 40, "height": 90}]     # centre format, pixels
    # or x1/y1/x2/y2 corners
```

## How the camera "looks around"

| `mode:=` | behaviour |
|----------|-----------|
| `step` (default) | stand at each of a 3×3 grid of positions and turn in 45° steps — systematic coverage |
| `random` | random viewpoints, 70 % aimed at an object so frames aren't empty |
| `sweep` | continuous patrol along a serpentine path while spinning |
| `manual` | you drive it: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cv_sim/camera_cmd` |

Switch live: `ros2 param set /camera_scanner mode random`. In `step`/`random` each view is held
until the model has been scored on a frame from it, so a slow (remote) model makes the run slower
rather than sparser. Re-randomize the objects any time: `ros2 service call /cv_sim/randomize_scene std_srvs/srv/Trigger`.

## How scoring works

Every object's position is known exactly, so for each frame the evaluator projects the objects
into the image using the camera pose *at that frame's timestamp* and compares with your detections
(IoU ≥ 0.5, greedy by confidence).

* Objects that are tiny (<10 px), <60 % in frame, >40 % hidden behind other objects, or beyond 6 m are
  **"don't care"** — missing them isn't a miss and detecting them isn't a false alarm.
  Tune in `config/cv_sim.yaml` (`cv_evaluator`).
* A detection with the wrong class on a real object is both a false alarm and a miss, and shows up
  in the confusion table. `class_agnostic:=true` scores localisation only.
* Frames taken while the camera is teleporting or the scene is being re-spawned are dropped and
  counted under "Frames not scored".
* Occlusion is approximated from the objects' projected boxes (nearer boxes cover farther ones);
  the walls are not considered.

## Make it match your robot (important)

1. **`config/objects.yaml` — class names.** `class_name` must equal what your model outputs, or
   everything scores as wrong. Run once with your model and read the labels on the live page.
2. **`config/objects.yaml` — shapes/sizes/colours are placeholders.** The public docs for 2026-27
   Override give no dimensions. Put real values in, or point `shape: mesh` at your CAD.
3. **`worlds/cv_field.world` — camera block.** Set resolution and `horizontal_fov` to your real camera,
   and `height` / `pitches_deg` in `config/cv_sim.yaml` to where it's mounted.
4. **Domain gap.** A model trained on real photos will not necessarily score like it does on real
   life when shown Gazebo renders — flat colours, no motion blur, no field clutter. Treat the
   absolute numbers with suspicion; the sim is most reliable for *comparing* model versions,
   finding viewing angles/distances where a model falls apart, and regression testing. To
   narrow the gap, use meshes/textures that look like your training data.

## Layout

```
ghost_cv_sim/
  config/        objects.yaml (catalog), scenarios/*.yaml, cv_sim.yaml (node params)
  worlds/        cv_field.world (field, walls, lights, camera)
  launch/        cv_sim.launch.py
  ghost_cv_sim/  scene_manager_node, camera_scanner_node, cv_detector_node, evaluator_node
                 + ROS-free core: geometry, scene, viewpoints, metrics, detectors
  test/          python3 -m unittest discover -s test -t .   (no ROS needed)
```

Scenarios: `scenario:=default | crowded | fixed_example` (or add a file to `config/scenarios/`).

## Topics

`/cv_sim/camera/image_raw` raw frames · `/cv/detections` JSON detections ·
`/cv/image_annotated` detector output · `/cv/eval_image` ground truth vs detections ·
`/cv/eval_stats` running metrics · `/cv_sim/ground_truth` · `/cv_sim/camera_pose` ·
services `/cv_sim/randomize_scene`, `/cv_sim/save_report`, `/cv_sim/reset_metrics`.
