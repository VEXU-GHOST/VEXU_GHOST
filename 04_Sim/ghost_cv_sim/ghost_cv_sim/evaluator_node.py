#!/usr/bin/env python3
"""Scores the CV model against simulator ground truth.

For every detection message it looks up (by the *image's* stamp, not "now"):
  * the camera pose at that instant            (/cv_sim/camera_pose, interpolated)
  * the scene that existed at that instant     (/cv_sim/ground_truth history)
projects every object into the image, decides which ones a good model should find
(`expected`) vs which are too small / cut off / occluded (`ignore`), and matches.

Frames taken while the camera is teleporting or the scene is being re-spawned are dropped
(counted under "Frames not scored" so nothing silently disappears).
"""
import bisect
import csv
import json
import math
import os
import time
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Header, String
from std_srvs.srv import Trigger

from ghost_cv_sim.geometry import slerp_quat
from ghost_cv_sim.metrics import Accumulator, det_to_xyxy, format_summary, match_frame
from ghost_cv_sim.ros_utils import bgr_to_image, image_to_bgr, stamp_to_sec
from ghost_cv_sim.scene import project_objects
from ghost_cv_sim.web_viewer import WebViewer

GREEN, CYAN, RED, YELLOW, GRAY = (0, 200, 0), (255, 220, 0), (50, 50, 255), (0, 230, 255), (140, 140, 140)


def _quat_to_yaw_pitch(q):
    x, y, z, w = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return yaw, pitch


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__("cv_evaluator")
        p = self.declare_parameter
        p("objects_file", "")
        p("detections_topic", "/cv/detections")
        p("image_topic", "/cv_sim/camera/image_raw")
        p("camera_info_topic", "/cv_sim/camera/camera_info")
        p("iou_threshold", 0.5)
        p("class_agnostic", False)         # True: only score "found an object", ignore class
        p("min_confidence", 0.0)
        p("min_box_px", 10)                # GT smaller than this (px) is ignored, not "missed"
        p("min_in_image", 0.6)             # GT less than this fraction in frame is ignored
        p("max_occluded", 0.4)             # GT more occluded than this is ignored
        p("max_range", 6.0)
        p("settle", 0.4)                   # sim seconds ignored after a camera jump
        p("jump_pre", 0.1)
        p("gt_settle", 0.4)                # sim seconds ignored after a scene change
        p("report_dir", "")
        p("max_failure_images", 300)
        p("web_port", 8090)                # 0 disables the browser viewer
        p("log_every", 10)

        with open(self.get_parameter("objects_file").value) as f:
            self._catalog = yaml.safe_load(f)["objects"]
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._acc = Accumulator()
        self._intr = None
        self._img_size = None
        self._pose_t, self._pose_v = [], []
        self._jumps = []
        self._gt_t, self._gt_v = [], []
        self._images = OrderedDict()
        self._drops = {}
        self._rows = []
        self._n_failure_imgs = 0
        self._report_dir = None
        self._last_eval_wall = 0.0
        self._last_raw_wall = 0.0
        self._last_saved_frames = -1

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, g("detections_topic"), self._on_det, 20)
        self.create_subscription(Image, g("image_topic"), self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self._on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/cv_sim/camera_pose", self._on_pose, 100)
        self.create_subscription(Float64, "/cv_sim/camera_jump", lambda m: self._jumps.append(m.data), 20)
        self.create_subscription(String, "/cv_sim/ground_truth", self._on_gt, latched)
        self._overlay_pub = self.create_publisher(Image, "/cv/eval_image", 1)
        self._stats_pub = self.create_publisher(String, "/cv/eval_stats", 1)
        self._done_pub = self.create_publisher(Float64, "/cv_sim/eval_done", 10)
        self.create_service(Trigger, "/cv_sim/save_report", self._srv_save)
        self.create_service(Trigger, "/cv_sim/reset_metrics", self._srv_reset)

        self._viewer = None
        port = int(g("web_port"))
        if port > 0:
            try:
                self._viewer = WebViewer(port)
                self.get_logger().info("Live results: http://localhost:%d" % port)
            except OSError as e:
                self.get_logger().warn("Web viewer disabled (port %d): %s" % (port, e))

    # ------------------------------------------------------------------ inputs
    def _on_info(self, msg):
        k = msg.k
        self._intr = (k[0], k[4], k[2], k[5])
        self._img_size = (msg.width, msg.height)

    def _on_pose(self, msg):
        q = msg.pose.orientation
        self._pose_t.append(stamp_to_sec(msg.header.stamp))
        self._pose_v.append((np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
                             np.array([q.x, q.y, q.z, q.w])))
        if len(self._pose_t) > 4000:
            del self._pose_t[:1000], self._pose_v[:1000]
        del self._jumps[:-200]

    def _on_gt(self, msg):
        d = json.loads(msg.data)
        self._gt_t.append(d["time"])
        self._gt_v.append(d["objects"])
        del self._gt_t[:-50], self._gt_v[:-50]

    def _on_image(self, msg):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        self._images[stamp_to_sec(msg.header.stamp)] = bgr
        while len(self._images) > 90:
            self._images.popitem(last=False)
        # Nothing scored recently: show raw frames + why, so a silent pipeline is debuggable.
        now = time.monotonic()
        if self._viewer and now - self._last_eval_wall > 1.5 and now - self._last_raw_wall > 0.2:
            self._last_raw_wall = now
            img = bgr.copy()
            why = max(self._drops, key=self._drops.get) if self._drops else "no detections received yet"
            cv2.putText(img, "not scoring: " + why, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 200, 255), 1, cv2.LINE_AA)
            self._push_viewer(img)

    # ---------------------------------------------------------------- lookups
    def _pose_at(self, t):
        ts = self._pose_t
        i = bisect.bisect_right(ts, t)
        if i == 0:
            return None
        if i == len(ts):
            return self._pose_v[-1] if t - ts[-1] <= 0.15 else None
        if ts[i] - ts[i - 1] > 0.25:
            return None
        a = (t - ts[i - 1]) / max(1e-9, ts[i] - ts[i - 1])
        (p0, q0), (p1, q1) = self._pose_v[i - 1], self._pose_v[i]
        return p0 + a * (p1 - p0), slerp_quat(q0, q1, a)

    def _gt_at(self, t):
        i = bisect.bisect_right(self._gt_t, t) - 1
        if i < 0:
            return None, "no_scene_yet"
        if self._gt_v[i] is None:
            return None, "scene_changing"
        if t < self._gt_t[i] + self.get_parameter("gt_settle").value:
            return None, "scene_settling"
        return self._gt_v[i], None

    def _drop(self, why):
        self._drops[why] = self._drops.get(why, 0) + 1

    # --------------------------------------------------------------- scoring
    def _on_det(self, msg):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        d = json.loads(msg.data)
        stamp = d["stamp"]
        if self._intr is None:
            return self._drop("no_camera_info")
        if any(j - g("jump_pre") <= stamp <= j + g("settle") for j in self._jumps[-50:]):
            return self._drop("camera_moving")
        scene, why = self._gt_at(stamp)
        if scene is None:
            return self._drop(why)
        pose = self._pose_at(stamp)
        if pose is None:
            return self._drop("no_pose_for_stamp")

        size = (d.get("width", self._img_size[0]), d.get("height", self._img_size[1]))
        gt = project_objects(scene, self._catalog, pose[0], pose[1], self._intr, size,
                             min_box_px=g("min_box_px"), min_in_image=g("min_in_image"),
                             max_occluded=g("max_occluded"), max_range=g("max_range"))
        dets = [x for x in d["detections"] if x["confidence"] >= g("min_confidence")]
        res = match_frame(gt, dets, iou_thr=g("iou_threshold"), class_agnostic=g("class_agnostic"))
        self._acc.add(gt, dets, res, latency_ms=d.get("inference_ms"))

        n_tp = sum(1 for v in res["gt_label"].values() if v == "tp")
        n_fn = sum(1 for v in res["gt_label"].values() if v == "fn")
        n_fp = sum(1 for v in res["det_label"].values() if v == "fp")
        overlay = self._draw(self._images.get(stamp), gt, dets, res, size)
        saved = ""
        if overlay is not None and (n_fn or n_fp):
            saved = self._save_failure(overlay, stamp)
        yaw, pitch = _quat_to_yaw_pitch(pose[1])
        self._rows.append([stamp, *np.round(pose[0], 3), round(yaw, 3), round(pitch, 3),
                           len(scene), n_tp + n_fn, n_tp, n_fp, n_fn, d.get("inference_ms"), saved])

        self._done_pub.publish(Float64(data=stamp))
        self._last_eval_wall = time.monotonic()
        stats = {"summary": self._acc.summary(), "drops": self._drops}
        if overlay is not None:
            self._overlay_pub.publish(bgr_to_image(overlay, Header(frame_id="cv_camera")))
            self._push_viewer(overlay)
        if self._viewer:
            self._viewer.update_stats(stats)
        if self._acc.frames % int(g("log_every")) == 0:
            self._stats_pub.publish(String(data=json.dumps(stats)))
            s = stats["summary"]
            self.get_logger().info("frames=%d  P=%s R=%s F1=%s  TP/FP/FN=%d/%d/%d" % (
                s["frames"], _fmt(s["precision"]), _fmt(s["recall"]), _fmt(s["f1"]),
                s["tp"], s["fp"], s["fn"]))

    def _draw(self, bgr, gt, dets, res, size):
        if bgr is None:
            return None
        img = bgr.copy()
        for gobj in gt:
            if gobj["box"] is None:
                continue
            x1, y1, x2, y2 = (int(round(v)) for v in gobj["box"])
            lab = res["gt_label"][gobj["id"]]
            if lab == "fn":
                cv2.rectangle(img, (x1, y1), (x2, y2), YELLOW, 2)
                cv2.putText(img, "MISS " + gobj["class_name"], (x1, min(size[1] - 4, y2 + 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, YELLOW, 1, cv2.LINE_AA)
            else:
                cv2.rectangle(img, (x1, y1), (x2, y2), GRAY if lab == "ignore" else GREEN, 1)
        for i, dd in enumerate(dets):
            x1, y1, x2, y2 = (int(round(v)) for v in det_to_xyxy(dd))
            lab = res["det_label"][i]
            col = {"tp": CYAN, "fp": RED, "ignored": GRAY}[lab]
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 2 if lab != "ignored" else 1)
            cv2.putText(img, "%s%s %.0f%%" % ("FP " if lab == "fp" else "", dd["class_name"],
                                              dd["confidence"] * 100),
                        (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.putText(img, "green=GT  cyan=TP  red=FP  yellow=MISS  gray=ignored", (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def _push_viewer(self, img):
        if self._viewer:
            ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self._viewer.update_frame(enc.tobytes())

    # ---------------------------------------------------------------- reporting
    def _ensure_report_dir(self):
        if self._report_dir is None:
            base = self.get_parameter("report_dir").value or os.path.join(
                os.environ.get("VEXU_HOME", os.path.expanduser("~")), "cv_sim_reports")
            self._report_dir = os.path.join(base, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(os.path.join(self._report_dir, "failures"), exist_ok=True)
        return self._report_dir

    def _save_failure(self, overlay, stamp):
        if self._n_failure_imgs >= int(self.get_parameter("max_failure_images").value):
            return ""
        rel = os.path.join("failures", "%010.3f.jpg" % stamp)
        cv2.imwrite(os.path.join(self._ensure_report_dir(), rel), overlay)
        self._n_failure_imgs += 1
        return rel

    def save_report(self):
        if self._acc.frames == 0:
            return None
        d = self._ensure_report_dir()
        summary = self._acc.summary()
        summary["frames_not_scored"] = self._drops
        summary["settings"] = {k: self.get_parameter(k).value for k in (
            "iou_threshold", "class_agnostic", "min_confidence", "min_box_px",
            "min_in_image", "max_occluded", "max_range")}
        with open(os.path.join(d, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        with open(os.path.join(d, "frames.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp", "cam_x", "cam_y", "cam_z", "cam_yaw", "cam_pitch", "n_objects",
                        "n_expected", "tp", "fp", "fn", "inference_ms", "failure_image"])
            w.writerows(self._rows)
        text = format_summary(summary)
        with open(os.path.join(d, "summary.txt"), "w") as f:
            f.write(text)
        self.get_logger().info("\n" + text + "\nReport saved to " + d)
        self._last_saved_frames = self._acc.frames
        return d

    def _srv_save(self, _req, res):
        d = self.save_report()
        res.success, res.message = (d is not None), (d or "no frames were scored yet")
        return res

    def _srv_reset(self, _req, res):
        self._acc.reset()
        self._rows, self._drops, self._n_failure_imgs, self._report_dir = [], {}, 0, None
        res.success, res.message = True, "metrics reset"
        return res

    def destroy_node(self):
        if self._acc.frames and self._acc.frames != self._last_saved_frames:
            self.save_report()
        if self._viewer:
            self._viewer.close()
        super().destroy_node()


def _fmt(v):
    return "-" if v is None else "%.3f" % v


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
