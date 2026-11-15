"""Pure-numpy geometry helpers (no ROS imports so they can be unit tested anywhere).

Conventions
-----------
* World frame: Gazebo's, x forward / y left / z up.
* Camera *link* frame (what Gazebo's camera sensor uses): x forward, y left, z up.
* Camera *optical* frame (what image pixels are in): z forward, x right, y down.
* Pitch is Gazebo's: positive pitch rotates the camera's nose DOWN.
"""
import math

import numpy as np


def quat_from_rpy(roll, pitch, yaw):
    """ZYX Euler angles -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def rot_from_quat(q):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_from_yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def look_at_yaw_pitch(eye, target):
    """Yaw/pitch (radians) so a camera at `eye` looks at `target` (roll = 0)."""
    d = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    return yaw, pitch


def intrinsics_from_fov(width, height, horizontal_fov):
    """Pinhole K from image size + horizontal FOV (radians). Square pixels."""
    fx = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


# link (x fwd, y left, z up) -> optical (x right, y down, z fwd)
_LINK_TO_OPTICAL = np.array([[0.0, -1.0, 0.0],
                             [0.0, 0.0, -1.0],
                             [1.0, 0.0, 0.0]])


def world_to_optical(points_w, cam_pos, cam_quat):
    """Transform Nx3 world points into the camera optical frame."""
    r_wc = rot_from_quat(np.asarray(cam_quat, dtype=float))
    p_link = (np.asarray(points_w, dtype=float) - np.asarray(cam_pos, dtype=float)) @ r_wc
    return p_link @ _LINK_TO_OPTICAL.T


def project_points(points_w, cam_pos, cam_quat, fx, fy, cx, cy, near=0.05):
    """Project Nx3 world points. Returns (uv Nx2, depth N, in_front N bool)."""
    p = world_to_optical(points_w, cam_pos, cam_quat)
    z = p[:, 2]
    in_front = z > near
    zs = np.where(in_front, z, 1.0)  # avoid divide-by-zero; masked by in_front
    u = fx * p[:, 0] / zs + cx
    v = fy * p[:, 1] / zs + cy
    return np.stack([u, v], axis=1), z, in_front


def slerp_quat(q0, q1, t):
    """Spherical interpolation between unit quaternions."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)
