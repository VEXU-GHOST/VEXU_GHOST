#!/bin/bash
# Publishes empty (all-zero) actuator commands so the serial node keeps talking to the
# V5 brain. Makes the brain GUI heartbeat blink without running the competition
# state machine. Pair with 11_Robots/ghost_override/launch/serial_plot.launch.py.
#
# Also broadcasts a fake map -> base_link TF that drives a counterclockwise circle around the
# field center (nose along the direction of travel), for the robot_pose minimap.
#
# Usage: scripts/debug/fake_actuator_cmds.sh [rate_hz]   (default 20)

RATE="${1:-20}"

# ghost_msgs lives in this checkout's workspace; source it so ros2 knows the type.
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

echo "Broadcasting fake map -> base_link TF"
python3 - <<'PY' &
import math, rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

rclpy.init()
node = rclpy.create_node('fake_robot_pose_tf')
broadcaster = TransformBroadcaster(node)
start = node.get_clock().now()

def tick():
    t = (node.get_clock().now() - start).nanoseconds * 1e-9 * 0.5  # rad
    yaw = t + math.pi / 2
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = 1.83 + math.cos(t)  # field center is 3 tiles = 1.83 m
    tf.transform.translation.y = 1.83 + math.sin(t)
    tf.transform.rotation.z = math.sin(yaw / 2)
    tf.transform.rotation.w = math.cos(yaw / 2)
    broadcaster.sendTransform(tf)

node.create_timer(0.05, tick)
rclpy.spin(node)
PY
TF_PID=$!
trap 'kill $TF_PID 2>/dev/null' EXIT

echo "Publishing empty /v5/actuator_command at ${RATE} Hz (Ctrl-C to stop)"
# digital_io must have 8 entries: the serial node indexes it [0..7] without a size check.
ros2 topic pub -r "$RATE" /v5/actuator_command ghost_msgs/msg/V5ActuatorCommand \
    "{digital_io: [false, false, false, false, false, false, false, false]}" > /dev/null
