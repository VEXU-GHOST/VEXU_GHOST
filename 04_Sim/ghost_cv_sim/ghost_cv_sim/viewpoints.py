"""Camera viewpoint generators. A view is (x, y, z, yaw, pitch) in the world frame.

Pitch is Gazebo's: positive = looking down. Pure python, no ROS.
"""
import math

from ghost_cv_sim.geometry import look_at_yaw_pitch


def step_views(field_half, margin, grid, z, yaw_step_deg, pitches_deg):
    """Systematic 'look around': stand at each grid station, rotate through all yaws."""
    lim = field_half - margin
    if grid <= 1:
        coords = [0.0]
    else:
        coords = [-lim + 2 * lim * i / (grid - 1) for i in range(grid)]
    n_yaw = max(1, int(round(360.0 / yaw_step_deg)))
    views = []
    for ix, x in enumerate(coords):
        for iy, y in enumerate(coords):
            for pitch in pitches_deg:
                for k in range(n_yaw):
                    views.append((x, y, z, -math.pi + 2 * math.pi * k / n_yaw,
                                  math.radians(pitch)))
    return views


def random_view(rng, scene, field_half, margin, z_range, pitch_range_deg,
                aim_prob=0.7, dist_range=(0.4, 2.0)):
    """Random viewpoint. With probability aim_prob it looks at a random scene object from
    a random bearing/distance (guarantees something to detect); otherwise anywhere."""
    lim = field_half - margin
    z = rng.uniform(*z_range)
    if scene and rng.random() < aim_prob:
        o = rng.choice(scene)
        for _ in range(20):
            d = rng.uniform(*dist_range)
            a = rng.uniform(-math.pi, math.pi)
            x, y = o["x"] + d * math.cos(a), o["y"] + d * math.sin(a)
            if abs(x) <= field_half - 0.1 and abs(y) <= field_half - 0.1:
                break
        else:
            x, y = max(-lim, min(lim, x)), max(-lim, min(lim, y))
        yaw, pitch = look_at_yaw_pitch((x, y, z), (o["x"], o["y"], 0.05))
        yaw += math.radians(rng.uniform(-15, 15))
        pitch += math.radians(rng.uniform(-8, 8))
        return (x, y, z, yaw, pitch)
    return (rng.uniform(-lim, lim), rng.uniform(-lim, lim), z,
            rng.uniform(-math.pi, math.pi), math.radians(rng.uniform(*pitch_range_deg)))


def serpentine_stations(field_half, margin, grid):
    """Waypoints for a continuous sweep: boustrophedon over a grid x grid lattice."""
    lim = field_half - margin
    coords = [0.0] if grid <= 1 else [-lim + 2 * lim * i / (grid - 1) for i in range(grid)]
    pts = []
    for i, x in enumerate(coords):
        col = coords if i % 2 == 0 else list(reversed(coords))
        pts.extend((x, y) for y in col)
    return pts
