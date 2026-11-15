"""Tests for the ROS-free core (geometry, scene, metrics, viewpoints). Run: python3 -m unittest"""
import math
import os
import random
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from ghost_cv_sim import metrics, scene, viewpoints
from ghost_cv_sim.geometry import (intrinsics_from_fov, look_at_yaw_pitch, project_points,
                                   quat_from_rpy, rot_from_quat, slerp_quat)

HERE = os.path.dirname(os.path.abspath(__file__))
CATALOG = yaml.safe_load(open(os.path.join(HERE, "..", "config", "objects.yaml")))["objects"]
W, H = 640, 480
INTR = intrinsics_from_fov(W, H, 1.309)


def cam(pos, yaw=0.0, pitch=0.0):
    return np.array(pos, float), quat_from_rpy(0.0, pitch, yaw)


class GeometryTest(unittest.TestCase):
    def test_point_ahead_hits_image_centre(self):
        p, q = cam((0, 0, 0.5))
        uv, z, front = project_points(np.array([[1.0, 0.0, 0.5]]), p, q, *INTR)
        self.assertTrue(front[0])
        self.assertAlmostEqual(z[0], 1.0)
        np.testing.assert_allclose(uv[0], [W / 2, H / 2], atol=1e-6)

    def test_left_is_smaller_u_and_up_is_smaller_v(self):
        p, q = cam((0, 0, 0.5))
        uv, _, _ = project_points(np.array([[1.0, 0.2, 0.5], [1.0, 0.0, 0.7]]), p, q, *INTR)
        self.assertLess(uv[0, 0], W / 2)      # +y (left) -> left of centre
        self.assertLess(uv[1, 1], H / 2)      # +z (up) -> above centre

    def test_positive_pitch_looks_down(self):
        p, q = cam((0, 0, 1.0), pitch=math.radians(45))
        uv, _, _ = project_points(np.array([[1.0, 0.0, 0.0]]), p, q, *INTR)  # 45 deg below
        np.testing.assert_allclose(uv[0], [W / 2, H / 2], atol=1e-4)

    def test_yaw_rotates_view(self):
        p, q = cam((0, 0, 0.5), yaw=math.pi / 2)
        uv, _, front = project_points(np.array([[0.0, 1.0, 0.5]]), p, q, *INTR)
        self.assertTrue(front[0])
        np.testing.assert_allclose(uv[0], [W / 2, H / 2], atol=1e-6)

    def test_behind_camera_not_in_front(self):
        p, q = cam((0, 0, 0.5))
        _, _, front = project_points(np.array([[-1.0, 0.0, 0.5]]), p, q, *INTR)
        self.assertFalse(front[0])

    def test_look_at_roundtrip(self):
        eye, tgt = (1.0, -0.5, 0.4), (0.2, 0.7, 0.05)
        yaw, pitch = look_at_yaw_pitch(eye, tgt)
        uv, z, front = project_points(np.array([tgt]), np.array(eye), quat_from_rpy(0, pitch, yaw), *INTR)
        self.assertTrue(front[0])
        np.testing.assert_allclose(uv[0], [W / 2, H / 2], atol=1e-4)

    def test_quat_unit_and_slerp(self):
        q0, q1 = quat_from_rpy(0, 0, 0), quat_from_rpy(0, 0, math.pi / 2)
        self.assertAlmostEqual(float(np.linalg.norm(q0)), 1.0)
        mid = slerp_quat(q0, q1, 0.5)
        np.testing.assert_allclose(rot_from_quat(mid), rot_from_quat(quat_from_rpy(0, 0, math.pi / 4)), atol=1e-9)


class SceneTest(unittest.TestCase):
    def test_every_catalog_entry_makes_valid_sdf(self):
        for key, spec in CATALOG.items():
            if spec["shape"] == "mesh":
                continue
            root = ET.fromstring(scene.object_sdf("o_" + key, spec))
            self.assertEqual(root.find("model").get("name"), "o_" + key)
            self.assertEqual(root.find("model/static").text, "true")
            self.assertGreaterEqual(len(root.findall("model/link/visual")), 1)

    def test_two_tone_pin_has_two_colours(self):
        sdf = scene.object_sdf("p", CATALOG["RedBluePin"])
        self.assertEqual(len(set(e.text for e in ET.fromstring(sdf).iter("ambient"))), 2)

    def test_generation_no_overlap_inside_field_and_seeded(self):
        counts = {"RedBluePin": [5, 5], "Cup": [10, 10]}
        a = scene.generate_scene(CATALOG, counts, 1.83, 0.15, random.Random(7))
        b = scene.generate_scene(CATALOG, counts, 1.83, 0.15, random.Random(7))
        self.assertEqual(a, b)
        self.assertEqual(len(a), 15)
        for o in a:
            self.assertLessEqual(abs(o["x"]), 1.83 - 0.15)
            self.assertLessEqual(abs(o["y"]), 1.83 - 0.15)
        for i, o in enumerate(a):
            for p in a[i + 1:]:
                ri = scene.object_extent(CATALOG[o["type"]])[0]
                rj = scene.object_extent(CATALOG[p["type"]])[0]
                self.assertGreaterEqual(math.hypot(o["x"] - p["x"], o["y"] - p["y"]), ri + rj)

    def test_unknown_type_is_a_clear_error(self):
        with self.assertRaises(KeyError):
            scene.generate_scene(CATALOG, {"Nope": 1}, 1.8, 0.1, random.Random(0))

    def _one(self, x, y, t="YellowPin", pos=(0, 0, 0.35), yaw=0.0, pitch=0.0):
        sc = scene.explicit_scene(CATALOG, [{"type": t, "x": x, "y": y}])
        p, q = cam(pos, yaw, pitch)
        return scene.project_objects(sc, CATALOG, p, q, INTR, (W, H))

    def test_object_ahead_is_expected_and_centred(self):
        r = self._one(0.8, 0.0)[0]
        self.assertEqual(r["status"], "expected")
        x1, y1, x2, y2 = r["box"]
        self.assertAlmostEqual((x1 + x2) / 2, W / 2, delta=1.0)
        self.assertGreater(y2 - y1, x2 - x1)          # a pin is taller than wide
        self.assertAlmostEqual(r["depth"], 0.8, delta=0.04)

    def test_object_behind_is_dropped_offscreen_ignored(self):
        self.assertEqual(self._one(-1.0, 0.0), [])
        self.assertEqual(self._one(0.5, 5.0), [])      # far off to the side

    def test_far_object_is_ignored_not_expected(self):
        self.assertEqual(self._one(7.0, 0.0)[0]["status"], "ignore")

    def test_occlusion_marks_the_hidden_one_ignored(self):
        sc = scene.explicit_scene(CATALOG, [{"type": "Cup", "x": 0.4, "y": 0.0},
                                            {"type": "YellowPin", "x": 1.2, "y": 0.0}])
        p, q = cam((0, 0, 0.1))
        rows = {r["id"]: r for r in scene.project_objects(sc, CATALOG, p, q, INTR, (W, H))}
        self.assertEqual(rows[0]["status"], "expected")
        self.assertGreater(rows[1]["occluded"], 0.4)
        self.assertEqual(rows[1]["status"], "ignore")


def gtrow(i, cls, box, status="expected", depth=1.0):
    return {"id": i, "class_name": cls, "box": box, "depth": depth, "status": status}


def det(cls, box, conf=0.9):
    return {"class_name": cls, "confidence": conf, "x1": box[0], "y1": box[1], "x2": box[2], "y2": box[3]}


class MetricsTest(unittest.TestCase):
    def test_iou(self):
        self.assertAlmostEqual(metrics.iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0)
        self.assertAlmostEqual(metrics.iou((0, 0, 10, 10), (5, 0, 15, 10)), 1 / 3)
        self.assertEqual(metrics.iou((0, 0, 1, 1), (2, 2, 3, 3)), 0.0)

    def test_center_format_conversion(self):
        self.assertEqual(metrics.det_to_xyxy({"x": 10, "y": 20, "width": 4, "height": 6}), (8, 17, 12, 23))

    def run_one(self, gt, dets, **kw):
        res = metrics.match_frame(gt, dets, **kw)
        acc = metrics.Accumulator()
        acc.add(gt, dets, res)
        return res, acc

    def test_tp_fp_fn(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10)), gtrow(1, "A", (50, 50, 60, 60))]
        dets = [det("A", (0, 0, 10, 10)), det("A", (100, 100, 110, 110))]
        res, acc = self.run_one(gt, dets)
        self.assertEqual((acc.tp, acc.fp, acc.fn), (1, 1, 1))
        self.assertEqual(res["gt_label"][1], "fn")
        self.assertEqual(res["det_label"][1], "fp")

    def test_wrong_class_is_fp_and_fn_and_recorded_as_confusion(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10))]
        res, acc = self.run_one(gt, [det("B", (0, 0, 10, 10))])
        self.assertEqual((acc.tp, acc.fp, acc.fn), (0, 1, 1))
        self.assertEqual(acc.confusion["A"]["B"], 1)
        _, acc2 = self.run_one(gt, [det("B", (0, 0, 10, 10))], class_agnostic=True)
        self.assertEqual((acc2.tp, acc2.fp, acc2.fn), (1, 0, 0))

    def test_ignored_gt_is_neither_miss_nor_false_alarm(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10), status="ignore")]
        _, acc = self.run_one(gt, [])
        self.assertEqual((acc.tp, acc.fp, acc.fn), (0, 0, 0))
        _, acc = self.run_one(gt, [det("A", (0, 0, 10, 10))])
        self.assertEqual((acc.tp, acc.fp, acc.fn), (0, 0, 0))

    def test_duplicate_detection_counts_one_fp(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10))]
        _, acc = self.run_one(gt, [det("A", (0, 0, 10, 10), 0.9), det("A", (0, 0, 10, 10), 0.8)])
        self.assertEqual((acc.tp, acc.fp, acc.fn), (1, 1, 0))

    def test_higher_confidence_wins_the_match(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10))]
        dets = [det("A", (1, 1, 11, 11), 0.4), det("A", (0, 0, 10, 10), 0.9)]
        res, _ = self.run_one(gt, dets)
        self.assertEqual(res["det_label"], {1: "tp", 0: "fp"})

    def test_perfect_model_ap_is_one(self):
        acc = metrics.Accumulator()
        for k in range(5):
            gt = [gtrow(0, "A", (0, 0, 10, 10), depth=0.3 + k)]
            d = [det("A", (0, 0, 10, 10))]
            acc.add(gt, d, metrics.match_frame(gt, d))
        s = acc.summary()
        self.assertAlmostEqual(s["map50"], 1.0)
        self.assertAlmostEqual(s["f1"], 1.0)
        self.assertEqual([b["gt"] for b in s["recall_by_distance"]], [1, 0, 1, 1, 2])

    def test_ap_penalises_confident_false_positive(self):
        acc = metrics.Accumulator()
        gt = [gtrow(0, "A", (0, 0, 10, 10))]
        d = [det("A", (100, 100, 110, 110), 0.99), det("A", (0, 0, 10, 10), 0.5)]
        acc.add(gt, d, metrics.match_frame(gt, d))
        self.assertAlmostEqual(acc.summary()["per_class"]["A"]["ap50"], 0.5)

    def test_summary_text_renders(self):
        gt = [gtrow(0, "A", (0, 0, 10, 10))]
        _, acc = self.run_one(gt, [det("B", (0, 0, 10, 10))])
        txt = metrics.format_summary(acc.summary())
        self.assertIn("CV MODEL REPORT", txt)
        self.assertIn("A", txt)


class ViewpointTest(unittest.TestCase):
    def test_step_views_cover_all_yaws_and_stations(self):
        v = viewpoints.step_views(1.83, 0.25, 3, 0.35, 45.0, [20.0])
        self.assertEqual(len(v), 3 * 3 * 8)
        self.assertEqual(len({(round(a[0], 3), round(a[1], 3)) for a in v}), 9)
        self.assertTrue(all(abs(a[0]) <= 1.83 - 0.25 + 1e-9 for a in v))

    def test_random_view_aimed_sees_its_target(self):
        sc = scene.explicit_scene(CATALOG, [{"type": "Cup", "x": 0.3, "y": -0.2}])
        rng = random.Random(3)
        seen = 0
        for _ in range(50):
            x, y, z, yaw, pitch = viewpoints.random_view(rng, sc, 1.83, 0.25, (0.25, 0.6), (5, 40), aim_prob=1.0)
            p, q = np.array([x, y, z]), quat_from_rpy(0, pitch, yaw)
            rows = scene.project_objects(sc, CATALOG, p, q, INTR, (W, H), min_in_image=0.0, max_occluded=1.0)
            seen += bool(rows)
        self.assertGreaterEqual(seen, 45)

    def test_serpentine_visits_every_station_once(self):
        pts = viewpoints.serpentine_stations(1.83, 0.25, 3)
        self.assertEqual(len(pts), 9)
        self.assertEqual(len(set(pts)), 9)


if __name__ == "__main__":
    unittest.main()
