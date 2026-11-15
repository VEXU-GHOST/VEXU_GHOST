#!/usr/bin/env python3
"""Runs a CV backend on the simulated camera and publishes detections.

Only the *latest* frame is processed (a slow remote model drops frames instead of building
a backlog). Each result carries the stamp of the image it came from, which is what lets the
evaluator score it against the camera pose at that exact moment.
"""
import json
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ghost_cv_sim.detectors import DEFAULT_KEY_FILE, DEFAULT_ROBOFLOW_MODEL, make_backend
from ghost_cv_sim.metrics import det_to_xyxy
from ghost_cv_sim.ros_utils import bgr_to_image, image_to_bgr, stamp_to_sec


def class_color(name):
    n = name.lower()
    if "red" in n:
        return (60, 60, 255)
    if "blue" in n:
        return (255, 120, 40)
    if "yellow" in n:
        return (0, 220, 255)
    return (200, 200, 200)


class CvDetectorNode(Node):
    def __init__(self):
        super().__init__("cv_detector")
        p = self.declare_parameter
        p("backend", "roboflow")                       # roboflow | ultralytics | custom | color_blob
        p("image_topic", "/cv_sim/camera/image_raw")
        p("detections_topic", "/cv/detections")
        p("annotated_topic", "/cv/image_annotated")
        p("confidence_threshold", 0.25)
        p("max_fps", 0.0)                              # 0 = as fast as the model answers
        # roboflow
        p("model_id", DEFAULT_ROBOFLOW_MODEL)
        p("api_url", "https://serverless.roboflow.com")
        p("api_key_file", DEFAULT_KEY_FILE)
        # ultralytics
        p("weights", "")
        p("imgsz", 640)
        p("device", "cpu")
        # custom
        p("custom_callable", "")
        # color_blob
        p("min_area", 80)

        names = ("model_id", "api_url", "api_key_file", "weights", "imgsz", "device",
                 "custom_callable", "min_area")
        kwargs = {n: self.get_parameter(n).value for n in names}
        kwargs["conf"] = self.get_parameter("confidence_threshold").value
        backend_name = self.get_parameter("backend").value
        try:
            self._backend = make_backend(backend_name, **kwargs)
        except Exception as e:  # noqa: BLE001 - surface any backend setup problem clearly
            self.get_logger().fatal("Could not start backend '%s': %s" % (backend_name, e))
            raise SystemExit(1)
        self.get_logger().info("Detector backend: %s" % backend_name)

        self._thr = float(self.get_parameter("confidence_threshold").value)
        self._min_period = 1.0 / float(self.get_parameter("max_fps").value) \
            if float(self.get_parameter("max_fps").value) > 0 else 0.0
        self._det_pub = self.create_publisher(String, self.get_parameter("detections_topic").value, 10)
        self._img_pub = self.create_publisher(Image, self.get_parameter("annotated_topic").value, 1)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, qos_profile_sensor_data)

        self._cv = threading.Condition()
        self._latest = None
        self._last_err = 0.0
        self._running = True
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    def _on_image(self, msg):
        with self._cv:
            self._latest = msg
            self._cv.notify()

    def _loop(self):
        last = 0.0
        while self._running:
            with self._cv:
                while self._latest is None and self._running:
                    self._cv.wait(timeout=0.5)
                msg, self._latest = self._latest, None
            if msg is None:
                continue
            wait = self._min_period - (time.monotonic() - last)
            if wait > 0:
                time.sleep(wait)
            last = time.monotonic()
            try:
                bgr = image_to_bgr(msg)
                t0 = time.monotonic()
                dets = [d for d in self._backend.infer(bgr) if d["confidence"] >= self._thr]
                ms = (time.monotonic() - t0) * 1000.0
            except Exception as e:  # noqa: BLE001 - keep running; remote models flake
                if time.monotonic() - self._last_err > 5.0:
                    self.get_logger().error("Inference failed: %s" % e)
                    self._last_err = time.monotonic()
                continue
            out = {"stamp": stamp_to_sec(msg.header.stamp), "width": msg.width,
                   "height": msg.height, "inference_ms": ms, "detections": dets}
            self._det_pub.publish(String(data=json.dumps(out)))
            self._img_pub.publish(bgr_to_image(self._annotate(bgr, dets), msg.header))

    @staticmethod
    def _annotate(bgr, dets):
        img = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = (int(round(v)) for v in det_to_xyxy(d))
            col = class_color(d["class_name"])
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
            cv2.putText(img, "%s %.0f%%" % (d["class_name"], d["confidence"] * 100),
                        (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        return img

    def destroy_node(self):
        self._running = False
        with self._cv:
            self._cv.notify_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CvDetectorNode()
    except SystemExit:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
