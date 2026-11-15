"""Object catalog, SDF generation, scene randomization and ground-truth projection.

No ROS imports: everything here is plain python/numpy so it is unit tested directly.
"""
import math
import random

import numpy as np

from ghost_cv_sim.geometry import project_points, rot_from_yaw

RING_SAMPLES = 16
FRUSTUM_SLICES = 8


# --------------------------------------------------------------------------- catalog
def object_extent(spec):
    """(max horizontal radius, height) of the object's footprint, used for spacing."""
    shape = spec.get("shape", "cylinder")
    if shape == "cylinder":
        return float(spec["radius"]), float(spec["height"])
    if shape == "frustum":
        return max(float(spec["radius_bottom"]), float(spec["radius_top"])), float(spec["height"])
    if shape in ("box", "mesh"):
        sx, sy, sz = (float(v) for v in spec["size"])
        return 0.5 * math.hypot(sx, sy), sz
    raise ValueError("unknown shape '%s'" % shape)


def local_shape_points(spec):
    """Sample points on the object's hull in its own frame (base centred at z=0).

    Round shapes use rim rings instead of an AABB so the projected 2D box is tight, which
    matches how a labeller would draw the box.
    """
    shape = spec.get("shape", "cylinder")
    if shape in ("cylinder", "frustum"):
        if shape == "cylinder":
            rb = rt = float(spec["radius"])
        else:
            rb, rt = float(spec["radius_bottom"]), float(spec["radius_top"])
        h = float(spec["height"])
        pts = []
        for k in range(RING_SAMPLES):
            a = 2 * math.pi * k / RING_SAMPLES
            pts.append((rb * math.cos(a), rb * math.sin(a), 0.0))
            pts.append((rt * math.cos(a), rt * math.sin(a), h))
        return np.array(pts)
    sx, sy, sz = (float(v) for v in spec["size"])
    return np.array([(x * sx / 2, y * sy / 2, z * sz)
                     for x in (-1, 1) for y in (-1, 1) for z in (0, 1)])


def _slices(spec):
    """Visual decomposition: list of (z_center, height, radius, rgb) for round shapes."""
    shape = spec.get("shape", "cylinder")
    h = float(spec["height"])
    bands = spec.get("bands") or [[1.0, spec.get("color", [0.5, 0.5, 0.5])]]
    total = sum(float(b[0]) for b in bands)
    out = []
    z0 = 0.0
    for frac, rgb in bands:
        bh = h * float(frac) / total
        n = 1 if shape == "cylinder" else max(1, int(round(FRUSTUM_SLICES * float(frac) / total)))
        for i in range(n):
            sh = bh / n
            zc = z0 + sh * (i + 0.5)
            if shape == "cylinder":
                r = float(spec["radius"])
            else:  # linear taper, sampled at slice centre
                t = zc / h
                r = float(spec["radius_bottom"]) * (1 - t) + float(spec["radius_top"]) * t
            out.append((zc, sh, r, rgb))
        z0 += bh
    return out


def _material(rgb, jitter):
    c = [min(1.0, max(0.0, float(v) + jitter)) for v in rgb]
    return ("<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>"
            "<specular>0.05 0.05 0.05 1</specular></material>" % (c[0], c[1], c[2], c[0], c[1], c[2]))


def object_sdf(name, spec, color_jitter=0.0):
    """Static SDF model for one object, at the origin. Static so it never gets knocked around.

    The world pose is NOT in the SDF: gazebo_ros_factory takes it from the spawn request's
    initial_pose, so scene_manager passes it there.
    """
    shape = spec.get("shape", "cylinder")
    visuals = []
    if shape in ("cylinder", "frustum"):
        for i, (zc, sh, r, rgb) in enumerate(_slices(spec)):
            visuals.append(
                '<visual name="v%d"><pose>0 0 %.5f 0 0 0</pose><geometry><cylinder>'
                "<radius>%.5f</radius><length>%.5f</length></cylinder></geometry>%s</visual>"
                % (i, zc, r, sh * 1.001, _material(rgb, color_jitter)))
    elif shape == "box":
        sx, sy, sz = spec["size"]
        visuals.append(
            '<visual name="v0"><pose>0 0 %.5f 0 0 0</pose><geometry><box><size>%.5f %.5f %.5f</size>'
            "</box></geometry>%s</visual>"
            % (sz / 2, sx, sy, sz, _material(spec.get("color", [0.5, 0.5, 0.5]), color_jitter)))
    elif shape == "mesh":
        sc = spec.get("scale", [1, 1, 1])
        visuals.append(
            '<visual name="v0"><geometry><mesh><uri>%s</uri><scale>%.5f %.5f %.5f</scale></mesh>'
            "</geometry></visual>" % (spec["uri"], sc[0], sc[1], sc[2]))
    else:
        raise ValueError("unknown shape '%s'" % shape)
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="%s"><static>true</static>'
            '<link name="link">%s</link></model></sdf>' % (name, "".join(visuals)))


# ------------------------------------------------------------------ scene generation
def generate_scene(catalog, counts, field_half, margin, rng, min_gap=0.01, max_tries=200):
    """Rejection-sample non-overlapping upright objects inside the field.

    counts: {catalog_key: n or [min, max]}.  Returns list of dicts:
    {id, type, class_name, x, y, yaw}.  Objects that cannot be placed are dropped.
    """
    placed = []
    lim = field_half - margin
    for key, n in counts.items():
        if key not in catalog:
            raise KeyError("scenario references '%s' which is not in the object catalog" % key)
        if isinstance(n, (list, tuple)):
            n = rng.randint(int(n[0]), int(n[1]))
        radius, _ = object_extent(catalog[key])
        for _ in range(int(n)):
            for _try in range(max_tries):
                x, y = rng.uniform(-lim, lim), rng.uniform(-lim, lim)
                if all(math.hypot(x - o["x"], y - o["y"]) >= radius + o["_r"] + min_gap
                       for o in placed):
                    placed.append({"type": key, "x": x, "y": y, "yaw": rng.uniform(-math.pi, math.pi),
                                   "_r": radius})
                    break
    out = []
    for i, o in enumerate(placed):
        out.append({"id": i, "type": o["type"],
                    "class_name": catalog[o["type"]].get("class_name", o["type"]),
                    "x": o["x"], "y": o["y"], "yaw": o["yaw"]})
    return out


def explicit_scene(catalog, objects):
    """Scene from an explicit scenario list: [{type, x, y, yaw?}, ...]."""
    out = []
    for i, o in enumerate(objects):
        if o["type"] not in catalog:
            raise KeyError("scenario references '%s' which is not in the object catalog" % o["type"])
        out.append({"id": i, "type": o["type"],
                    "class_name": catalog[o["type"]].get("class_name", o["type"]),
                    "x": float(o["x"]), "y": float(o["y"]), "yaw": float(o.get("yaw", 0.0))})
    return out


# ------------------------------------------------------------- ground-truth projection
def project_objects(scene, catalog, cam_pos, cam_quat, intr, image_size,
                    min_box_px=10, min_in_image=0.6, max_occluded=0.4, max_range=6.0):
    """Project every scene object into the camera image.

    Returns one dict per object that lands in front of the camera:
      id, class_name, box=(x1,y1,x2,y2) unclipped pixels, depth, status
    status is 'expected' (a good model should find it) or 'ignore' (too small / mostly
    out of frame / mostly occluded / too far): ignored objects are neither misses nor
    false-positive sources when matched.
    """
    fx, fy, cx, cy = intr
    w, h = image_size
    rows = []
    for obj in scene:
        spec = catalog[obj["type"]]
        local = local_shape_points(spec)
        pts_w = local @ rot_from_yaw(obj["yaw"]).T + np.array([obj["x"], obj["y"], 0.0])
        uv, z, front = project_points(pts_w, cam_pos, cam_quat, fx, fy, cx, cy)
        if not front.all():  # partly behind / too near the camera: unusable box
            if not front.any():
                continue
            rows.append({"id": obj["id"], "class_name": obj["class_name"], "box": None,
                         "depth": float(np.mean(z[front])), "status": "ignore"})
            continue
        x1, y1 = float(uv[:, 0].min()), float(uv[:, 1].min())
        x2, y2 = float(uv[:, 0].max()), float(uv[:, 1].max())
        area = max(1e-6, (x2 - x1) * (y2 - y1))
        cx1, cy1, cx2, cy2 = max(0.0, x1), max(0.0, y1), min(w, x2), min(h, y2)
        inside = max(0.0, cx2 - cx1) * max(0.0, cy2 - cy1)
        if inside <= 0.0:
            continue  # completely off-screen
        rows.append({"id": obj["id"], "class_name": obj["class_name"],
                     "box": (x1, y1, x2, y2), "depth": float(z.mean()),
                     "in_image": inside / area})
    _mark_occlusion(rows, image_size)
    out = []
    for r in rows:
        if r["box"] is None:
            out.append(r)
            continue
        x1, y1, x2, y2 = r["box"]
        ok = (r["in_image"] >= min_in_image and r["occluded"] <= max_occluded
              and min(x2 - x1, y2 - y1) >= min_box_px and r["depth"] <= max_range)
        r["status"] = "expected" if ok else "ignore"
        out.append(r)
    return out


def _mark_occlusion(rows, image_size, grid=96):
    """Approximate occlusion: fraction of each box covered by nearer objects' boxes."""
    w, h = image_size
    sx, sy = grid / float(w), grid / float(h)
    boxed = sorted((r for r in rows if r["box"] is not None), key=lambda r: r["depth"])
    mask = np.zeros((grid, grid), dtype=bool)
    for r in boxed:
        x1, y1, x2, y2 = r["box"]
        gx1, gx2 = int(max(0, math.floor(x1 * sx))), int(min(grid, math.ceil(x2 * sx)))
        gy1, gy2 = int(max(0, math.floor(y1 * sy))), int(min(grid, math.ceil(y2 * sy)))
        region = mask[gy1:gy2, gx1:gx2]
        r["occluded"] = float(region.mean()) if region.size else 0.0
        mask[gy1:gy2, gx1:gx2] = True
    for r in rows:
        r.setdefault("occluded", 0.0)


def make_rng(seed):
    return random.Random(None if seed is None or int(seed) < 0 else int(seed))
