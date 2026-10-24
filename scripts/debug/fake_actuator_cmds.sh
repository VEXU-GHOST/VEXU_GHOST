#!/bin/bash
# Publishes empty (all-zero) actuator commands so the serial node keeps talking to the
# V5 brain. Makes the brain GUI heartbeat blink without running the competition
# state machine. Pair with 11_Robots/ghost_override/launch/serial_plot.launch.py.
#
# Usage: scripts/debug/fake_actuator_cmds.sh [rate_hz]   (default 20)

RATE="${1:-20}"

# ghost_msgs lives in this checkout's workspace; source it so ros2 knows the type.
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
echo "Publishing empty /v5/actuator_command at ${RATE} Hz (Ctrl-C to stop)"
# digital_io must have 8 entries: the serial node indexes it [0..7] without a size check.
exec ros2 topic pub -r "$RATE" /v5/actuator_command ghost_msgs/msg/V5ActuatorCommand \
    "{digital_io: [false, false, false, false, false, false, false, false]}" > /dev/null
