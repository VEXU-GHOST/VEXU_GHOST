#!/usr/bin/env python3
"""Spawns / randomizes the game objects in Gazebo and publishes the ground truth.

Ground truth goes out on /cv_sim/ground_truth (latched JSON). Objects are static, so their
poses are exactly what was spawned - no need to query Gazebo.

Services (std_srvs/Trigger): /cv_sim/randomize_scene, /cv_sim/clear_scene
"""
import json
import math
import time

import rclpy
import yaml
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ghost_cv_sim.scene import explicit_scene, generate_scene, make_rng, object_sdf

PREFIX = "cvobj_"


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        p = self.declare_parameter
        p("objects_file", "")
        p("scenario_file", "")
        p("field_half", 1.83)            # 12 ft field -> 3.66 m square
        p("edge_margin", 0.15)
        p("seed", -1)                    # -1 = different every run; >=0 = reproducible
        p("color_jitter", 0.04)          # +-brightness jitter per object (robustness)
        p("spawn_on_start", True)
        p("spawn_timeout", 30.0)

        with open(self.get_parameter("objects_file").value) as f:
            self._catalog = yaml.safe_load(f)["objects"]
        with open(self.get_parameter("scenario_file").value) as f:
            self._scenario = yaml.safe_load(f)
        self._half = float(self.get_parameter("field_half").value)
        self._margin = float(self.get_parameter("edge_margin").value)
        self._jitter = float(self.get_parameter("color_jitter").value)
        self._rng = make_rng(self.get_parameter("seed").value)
        self._spawned = []
        self._scene = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Callbacks here block on Gazebo service replies, which only works if the reply
        # callbacks can run concurrently with the blocked callback -> reentrant group.
        cbg = ReentrantCallbackGroup()
        self._gt_pub = self.create_publisher(String, "/cv_sim/ground_truth", latched)
        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity", callback_group=cbg)
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity", callback_group=cbg)
        self.create_service(Trigger, "/cv_sim/randomize_scene", self._srv_randomize, callback_group=cbg)
        self.create_service(Trigger, "/cv_sim/clear_scene", self._srv_clear, callback_group=cbg)

        if self.get_parameter("spawn_on_start").value:
            self._start_timer = self.create_timer(0.5, self._startup, callback_group=cbg)

    # ------------------------------------------------------------------ helpers
    def _startup(self):
        self._start_timer.cancel()
        self.randomize()

    def _wait(self, fut, what):
        deadline = time.monotonic() + float(self.get_parameter("spawn_timeout").value)
        while not fut.done():
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for %s" % what)
            time.sleep(0.01)
        return fut.result()

    def _build_scene(self):
        if "objects" in self._scenario:
            return explicit_scene(self._catalog, self._scenario["objects"])
        return generate_scene(self._catalog, self._scenario["counts"], self._half, self._margin,
                              self._rng, min_gap=float(self._scenario.get("min_gap", 0.01)))

    def clear(self):
        futs = []
        for name in self._spawned:
            req = DeleteEntity.Request()
            req.name = name
            futs.append(self._delete_cli.call_async(req))
        for f in futs:
            self._wait(f, "delete_entity")
        self._spawned = []
        self._scene = []

    def randomize(self):
        if not self._spawn_cli.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("/spawn_entity not available - is Gazebo running with gazebo_ros_factory?")
        self._publish_gt(changing=True)  # tells the evaluator to drop frames until we're done
        self.clear()
        scene = self._build_scene()
        futs = []
        for o in scene:
            req = SpawnEntity.Request()
            req.name = "%s%d" % (PREFIX, o["id"])
            req.xml = object_sdf(req.name, self._catalog[o["type"]],
                                 color_jitter=self._rng.uniform(-self._jitter, self._jitter))
            req.initial_pose = Pose()
            req.initial_pose.position.x, req.initial_pose.position.y = o["x"], o["y"]
            req.initial_pose.orientation.z = math.sin(o["yaw"] / 2)
            req.initial_pose.orientation.w = math.cos(o["yaw"] / 2)
            req.reference_frame = "world"
            futs.append((req.name, self._spawn_cli.call_async(req)))
        for name, f in futs:
            res = self._wait(f, "spawn_entity")
            if res.success:
                self._spawned.append(name)
            else:
                self.get_logger().warn("spawn %s failed: %s" % (name, res.status_message))
        self._scene = scene
        self._publish_gt()
        self.get_logger().info("Scene ready: %d objects" % len(scene))

    def _publish_gt(self, changing=False):
        self._gt_pub.publish(String(data=json.dumps(
            {"time": self.get_clock().now().nanoseconds * 1e-9,
             "objects": None if changing else self._scene})))

    # ----------------------------------------------------------------- services
    def _srv_randomize(self, _req, res):
        try:
            self.randomize()
            res.success, res.message = True, "%d objects" % len(self._scene)
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res

    def _srv_clear(self, _req, res):
        self.clear()
        self._publish_gt()
        res.success, res.message = True, "cleared"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    # Services block while waiting on Gazebo, so they need their own threads.
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
