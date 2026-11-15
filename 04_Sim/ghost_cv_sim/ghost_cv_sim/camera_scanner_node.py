#!/usr/bin/env python3
"""Moves the free-flying simulator camera around the field ("looking around").

Modes (change live with `ros2 param set /camera_scanner mode random`):
  step    stand at each station of a grid, rotate through all yaw angles, then next station
  random  random viewpoints, mostly aimed at objects in the scene
  sweep   continuous patrol: drive a serpentine path while slowly spinning
  manual  drive it yourself: geometry_msgs/Twist on /cv_sim/camera_cmd
          (linear x/y/z = forward/left/up m/s, angular.z = yaw rate, angular.y = pitch rate,
           positive pitch looks down).  e.g. `ros2 run teleop_twist_keyboard teleop_twist_keyboard
           --ros-args -r cmd_vel:=/cv_sim/camera_cmd`

step/random hold each view until the evaluator has scored a frame from it (so a slow model
simply makes the run slower, not sparser). After `views_per_scene` views the scene is
re-randomized; after `num_scenes` scenes the report is saved and this node exits.

Publishes /cv_sim/camera_pose (PoseStamped, sim time) and /cv_sim/camera_jump (Float64,
sim time of each teleport) so the evaluator can match each image to the exact pose.
"""
import json
import math

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from ghost_cv_sim.geometry import quat_from_rpy
from ghost_cv_sim.scene import make_rng
from ghost_cv_sim.viewpoints import random_view, serpentine_stations, step_views


class CameraScanner(Node):
    def __init__(self):
        super().__init__("camera_scanner")
        p = self.declare_parameter
        p("mode", "step")
        p("entity_name", "cv_camera")
        p("set_state_service", "/cv_sim/set_entity_state")
        p("pose_rate", 20.0)
        p("field_half", 1.83)
        p("edge_margin", 0.25)
        p("height", 0.35)                      # camera height for step / sweep / manual start
        p("z_range", [0.25, 0.6])              # random mode
        p("pitch_deg", 20.0)                   # sweep / manual start (positive = look down)
        p("pitch_range_deg", [5.0, 40.0])      # random mode, unaimed views
        p("seed", -1)
        # step
        p("station_grid", 3)
        p("yaw_step_deg", 45.0)
        p("pitches_deg", [20.0])
        # random
        p("aim_prob", 0.7)
        # sweep
        p("sweep_speed", 0.25)
        p("sweep_yaw_rate_deg", 30.0)
        # episode control (step / random)
        p("views_per_scene", 0)                # 0 = step: every view; random: 8
        p("num_scenes", 0)                     # 0 = run forever
        p("wait_for_eval", True)
        p("settle", 0.4)                       # sim seconds to ignore frames after a jump
        p("min_dwell", 0.3)
        p("max_dwell", 30.0)

        self._rng = make_rng(self.get_parameter("seed").value)
        self._set_cli = self.create_client(SetEntityState, self.get_parameter("set_state_service").value)
        self._pose_pub = self.create_publisher(PoseStamped, "/cv_sim/camera_pose", 50)
        self._jump_pub = self.create_publisher(Float64, "/cv_sim/camera_jump", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/cv_sim/ground_truth", self._on_gt, latched)
        self.create_subscription(Float64, "/cv_sim/eval_done", self._on_eval_done, 10)
        self.create_subscription(Twist, "/cv_sim/camera_cmd", self._on_cmd, 10)
        self._rand_cli = self.create_client(Trigger, "/cv_sim/randomize_scene")
        self._save_cli = self.create_client(Trigger, "/cv_sim/save_report")

        self._scene = []
        self._gt_ready = False
        self._eval_stamp = -1.0
        self._pose = (0.0, 0.0, float(self.get_parameter("height").value), 0.0,
                      math.radians(float(self.get_parameter("pitch_deg").value)))
        self._inflight = None
        self._warned = None
        # view state machine (step / random)
        self._mode = None
        self._views = []
        self._view_idx = 0
        self._views_in_scene = 0
        self._scenes_done = 0
        self._view_start = None
        self._jump_time = -1.0
        self._pending = None          # future for randomize / save_report
        self._state = "run"           # run | randomizing | saving
        self.done = False
        # sweep / manual
        self._stations = []
        self._station_i = 0
        self._cmd = Twist()
        self._cmd_time = None
        self._last_tick = None

        self.create_timer(1.0 / float(self.get_parameter("pose_rate").value), self._tick)
        self.get_logger().info("Camera scanner ready (mode=%s)" % self.get_parameter("mode").value)

    # ------------------------------------------------------------------ callbacks
    def _on_gt(self, msg):
        objs = json.loads(msg.data)["objects"]
        self._gt_ready = objs is not None   # None = scene is being re-spawned
        self._scene = objs or []

    def _on_eval_done(self, msg):
        self._eval_stamp = max(self._eval_stamp, msg.data)

    def _on_cmd(self, msg):
        self._cmd = msg
        self._cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _f(self, name):
        return float(self.get_parameter(name).value)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ main tick
    def _tick(self):
        now = self._now()
        if not self._set_cli.service_is_ready():
            if self._warned is None or now - self._warned > 5.0:
                self._warned = now
                self.get_logger().warn("Waiting for %s (is Gazebo up with gazebo_ros_state?)"
                                       % self._set_cli.srv_name)
            return
        dt = 0.0 if self._last_tick is None else max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        mode = self.get_parameter("mode").value
        if mode != self._mode:
            self._enter_mode(mode)

        if self._state != "run":
            self._poll_pending()
        elif mode in ("step", "random"):
            self._tick_episode(now)
        elif mode == "sweep":
            self._tick_sweep(dt)
        elif mode == "manual":
            self._tick_manual(now, dt)
        self._apply_pose(now)

    def _enter_mode(self, mode):
        if mode not in ("step", "random", "sweep", "manual"):
            self.get_logger().error("Unknown mode '%s'" % mode)
            self._mode = mode
            return
        self._mode = mode
        self._views, self._view_idx, self._views_in_scene = [], 0, 0
        self._view_start = None
        if mode == "step":
            self._views = step_views(self._f("field_half"), self._f("edge_margin"),
                                     int(self._f("station_grid")), self._f("height"),
                                     self._f("yaw_step_deg"),
                                     list(self.get_parameter("pitches_deg").value))
        elif mode == "sweep":
            self._stations = serpentine_stations(self._f("field_half"), self._f("edge_margin"),
                                                 int(self._f("station_grid")))
            self._station_i = 0
            x, y = self._stations[0]
            self._pose = (x, y, self._f("height"), 0.0, math.radians(self._f("pitch_deg")))
        self.get_logger().info("Mode: %s" % mode)

    # ------------------------------------------------------------- step / random
    def _views_per_scene(self):
        n = int(self._f("views_per_scene"))
        if n > 0:
            return n
        return len(self._views) if self._mode == "step" else 8

    def _tick_episode(self, now):
        if not self._gt_ready:
            return  # scene not spawned yet
        if self._view_start is not None and not self._view_finished(now):
            return
        if self._view_start is not None:
            self._views_in_scene += 1
            if self._views_in_scene >= self._views_per_scene():
                self._views_in_scene = 0
                self._scenes_done += 1
                n_scenes = int(self._f("num_scenes"))
                if n_scenes > 0 and self._scenes_done >= n_scenes:
                    self.get_logger().info("All %d scenes done - saving report" % n_scenes)
                    self._pending = self._save_cli.call_async(Trigger.Request())
                    self._state = "saving"
                    return
                self.get_logger().info("Scene %d done - randomizing" % self._scenes_done)
                self._pending = self._rand_cli.call_async(Trigger.Request())
                self._state = "randomizing"
                self._view_start = None
                return
        self._next_view(now)

    def _view_finished(self, now):
        elapsed = now - self._view_start
        if elapsed >= self._f("max_dwell"):
            self.get_logger().warn("No scored frame within max_dwell - is the detector running?")
            return True
        if elapsed < self._f("min_dwell") + self._f("settle"):
            return False
        if not self.get_parameter("wait_for_eval").value:
            return True
        return self._eval_stamp >= self._jump_time + self._f("settle")

    def _next_view(self, now):
        if self._mode == "step":
            self._pose = self._views[self._view_idx % len(self._views)]
            self._view_idx += 1
        else:
            self._pose = random_view(
                self._rng, self._scene, self._f("field_half"), self._f("edge_margin"),
                tuple(self.get_parameter("z_range").value),
                tuple(self.get_parameter("pitch_range_deg").value), self._f("aim_prob"))
        self._view_start = now
        self._jump_time = now
        self._jump_pub.publish(Float64(data=now))

    def _poll_pending(self):
        if self._pending is None or not self._pending.done():
            return
        res = self._pending.result()
        if not res.success:
            self.get_logger().error("Service call failed: %s" % res.message)
        self._pending = None
        if self._state == "saving":
            self.get_logger().info("Report: %s" % res.message)
            self.done = True
        self._state = "run"

    # -------------------------------------------------------------- sweep / manual
    def _tick_sweep(self, dt):
        x, y, z, yaw, pitch = self._pose
        tx, ty = self._stations[self._station_i]
        d = math.hypot(tx - x, ty - y)
        step = self._f("sweep_speed") * dt
        if d <= step:
            x, y = tx, ty
            self._station_i = (self._station_i + 1) % len(self._stations)
        else:
            x, y = x + (tx - x) / d * step, y + (ty - y) / d * step
        yaw = (yaw + math.radians(self._f("sweep_yaw_rate_deg")) * dt + math.pi) % (2 * math.pi) - math.pi
        self._pose = (x, y, z, yaw, pitch)

    def _tick_manual(self, now, dt):
        c = self._cmd if (self._cmd_time is not None and now - self._cmd_time < 0.5) else Twist()
        x, y, z, yaw, pitch = self._pose
        x += (c.linear.x * math.cos(yaw) - c.linear.y * math.sin(yaw)) * dt
        y += (c.linear.x * math.sin(yaw) + c.linear.y * math.cos(yaw)) * dt
        z += c.linear.z * dt
        yaw += c.angular.z * dt
        pitch += c.angular.y * dt
        lim = self._f("field_half") + 0.5
        self._pose = (max(-lim, min(lim, x)), max(-lim, min(lim, y)), max(0.05, min(2.0, z)),
                      (yaw + math.pi) % (2 * math.pi) - math.pi,
                      max(-math.pi / 2 + 0.05, min(math.pi / 2 - 0.05, pitch)))

    # ----------------------------------------------------------------- gazebo I/O
    def _apply_pose(self, now):
        x, y, z, yaw, pitch = self._pose
        q = quat_from_rpy(0.0, pitch, yaw)
        if self._inflight is None or self._inflight.done():
            req = SetEntityState.Request()
            st = EntityState()
            st.name = self.get_parameter("entity_name").value
            st.reference_frame = "world"
            st.pose.position.x, st.pose.position.y, st.pose.position.z = x, y, z
            st.pose.orientation.x, st.pose.orientation.y = float(q[0]), float(q[1])
            st.pose.orientation.z, st.pose.orientation.w = float(q[2]), float(q[3])
            req.state = st
            self._inflight = self._set_cli.call_async(req)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
        msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraScanner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
