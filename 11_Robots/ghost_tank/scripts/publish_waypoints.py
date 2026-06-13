#!/usr/bin/env python3
"""Parse GeneratePlannerPath waypoints from a BT XML and publish them as markers.

Reads every <GeneratePlannerPath end_x_tiles=.. end_y_tiles=.. end_theta_deg=..>
node from a behavior-tree XML, converts tiles -> meters and heading -> a yaw
quaternion, and publishes the set as a visualization_msgs/MarkerArray (map frame).
Each waypoint is an arrow colored on a green(start) -> red(finish) gradient, with
a small text label showing its index.

Usage:
    ros2 run ghost_tank publish_waypoints.py            # defaults below
    ./publish_waypoints.py --xml <file> --topic /waypoint_markers --frame map --once
"""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# 1 VEX tile = 24 in = 24 * 2.54 / 100 m
TILES_TO_METERS = 24.0 * 2.54 / 100.0  # 0.6096

DEFAULT_XML = Path(__file__).resolve().parents[1] / "config" / "bt_isolation_inky.xml"


def yaw_to_quat(yaw_rad):
    """Return a geometry_msgs/Quaternion for a yaw-only rotation."""
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def lerp_green_to_red(t):
    """Interpolate green (t=0) -> red (t=1) through yellow. t in [0, 1]."""
    return ColorRGBA(r=float(t), g=float(1.0 - t), b=0.0, a=1.0)


def parse_points(spec, fixed):
    """Parse a 'x,y,theta x,y,theta ...' string (tiles, degrees) into waypoints.

    fixed=True -> black; fixed=False -> green->red gradient.
    """
    waypoints = []
    for triple in spec.split():
        x_tiles, y_tiles, theta_deg = (float(v) for v in triple.split(","))
        waypoints.append((
            x_tiles * TILES_TO_METERS,
            y_tiles * TILES_TO_METERS,
            math.radians(theta_deg),
            fixed,
        ))
    return waypoints


def parse_waypoints(xml_path):
    """Extract (x_m, y_m, yaw_rad) tuples from every GeneratePlannerPath node."""
    root = ET.parse(xml_path).getroot()
    waypoints = []
    for node in root.iter("GeneratePlannerPath"):
        x_tiles = float(node.attrib["end_x_tiles"])
        y_tiles = float(node.attrib["end_y_tiles"])
        theta_deg = float(node.attrib["end_theta_deg"])
        waypoints.append((
            x_tiles * TILES_TO_METERS,
            y_tiles * TILES_TO_METERS,
            math.radians(theta_deg),
            False,  # XML path = middle points -> gradient
        ))
    return waypoints


def build_marker_array(node, waypoints, frame):
    """waypoints: list of (x_m, y_m, yaw_rad, fixed). Fixed (XML) -> black;
    ad-hoc --points -> green(start)->red(finish) gradient."""
    msg = MarkerArray()
    stamp = node.get_clock().now().to_msg()
    grad = [w for w in waypoints if not w[3]]
    n_grad = len(grad)
    grad_idx = 0
    for i, (x, y, yaw, fixed) in enumerate(waypoints):
        if fixed:
            color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        else:
            t = grad_idx / (n_grad - 1) if n_grad > 1 else 0.0
            color = lerp_green_to_red(t)
            grad_idx += 1

        arrow = Marker()
        arrow.header.frame_id = frame
        arrow.header.stamp = stamp
        arrow.ns = "waypoints"
        arrow.id = i
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.orientation = yaw_to_quat(yaw)
        arrow.scale.x = 0.25  # length
        arrow.scale.y = 0.05  # shaft width
        arrow.scale.z = 0.05  # head width
        arrow.color = color
        msg.markers.append(arrow)

        # Only label the fixed grid points, not the middle gradient ones.
        if fixed:
            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = "waypoint_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.15
            label.scale.z = 0.12  # text height
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            x_tiles = x / TILES_TO_METERS
            y_tiles = y / TILES_TO_METERS
            label.text = f"({x_tiles:g}, {y_tiles:g}) {math.degrees(yaw):.0f}deg"
            msg.markers.append(label)
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", default=str(DEFAULT_XML), help="behavior-tree XML path (gradient middle waypoints)")
    parser.add_argument(
        "--fixed",
        default="0,6,90 6,0,0",
        help="fixed (black) waypoints in tiles/degrees, e.g. \"0,6,90 6,0,0\"",
    )
    parser.add_argument(
        "--points",
        default=None,
        help="ad-hoc waypoints in tiles/degrees (green->red gradient)",
    )
    parser.add_argument("--topic", default="/waypoint_markers", help="MarkerArray topic to publish")
    parser.add_argument("--frame", default="map", help="header frame_id")
    parser.add_argument("--once", action="store_true", help="publish a single message and exit")
    parser.add_argument("--rate", type=float, default=1.0, help="publish rate (Hz) when not --once")
    args = parser.parse_args()

    # Fixed/black waypoints come from --fixed (and/or --xml); --points are gradient.
    waypoints = []
    sources = []
    if args.xml:
        waypoints += parse_waypoints(args.xml)
        sources.append(args.xml)
    if args.fixed:
        waypoints += parse_points(args.fixed, fixed=True)
        sources.append(f"--fixed '{args.fixed}'")
    if args.points:
        waypoints += parse_points(args.points, fixed=False)
        sources.append(f"--points '{args.points}'")
    source = " + ".join(sources)
    if not waypoints:
        raise SystemExit("No waypoints found (provide --xml, --fixed, or --points)")

    rclpy.init()
    node = rclpy.create_node("publish_waypoints")
    pub = node.create_publisher(MarkerArray, args.topic, 10)

    node.get_logger().info(
        f"Publishing {len(waypoints)} waypoints from {source} to {args.topic} "
        f"(frame={args.frame}, green=start -> red=finish)"
    )
    for i, (x, y, yaw, fixed) in enumerate(waypoints):
        node.get_logger().info(
            f"  [{i}] x={x:.4f} y={y:.4f} yaw={math.degrees(yaw):.1f} deg "
            f"({'fixed/black' if fixed else 'gradient'})"
        )

    try:
        if args.once:
            # Give discovery a moment so the single publish lands on subscribers.
            for _ in range(10):
                pub.publish(build_marker_array(node, waypoints, args.frame))
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            timer_period = 1.0 / args.rate
            node.create_timer(
                timer_period,
                lambda: pub.publish(build_marker_array(node, waypoints, args.frame)),
            )
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
