# Quantized peer-robot state exchanged between robots over VEXlink.
#
# This message IS the on-link payload: every field is already wire-format (quantized), so it
# serializes to exactly ghost_v5_interfaces::inter_robot::OTHER_ROBOT_PACKET_SIZE (7) bytes with no
# padding (see ghost_ros_interfaces::msg_helpers::packOtherRobot / unpackOtherRobot). The V5 brain
# relays these bytes opaquely; only ROS nodes interpret the fields.

# status values. UNKNOWN is 0 so a zeroed / stale slot reads as "no data".
uint8 STATUS_UNKNOWN=0
uint8 STATUS_COLLECTING=1
uint8 STATUS_SCORING=2
uint8 STATUS_PARKING=3
uint8 STATUS_STUCK=4
# Special: version announcement (sent at ~0.5 Hz). target_x carries the protocol version number;
# x/y/theta/target_y are not a pose in this case and must be ignored by the receiver.
uint8 STATUS_VERSION=5

uint8 x          # map->base_link x, 5 cm / unit
uint8 y          # map->base_link y, 5 cm / unit
uint8 theta      # heading, 0..255 maps to [0, 2*pi)
uint8 status     # one of STATUS_*
uint8 target_x   # intended target x (5 cm / unit); protocol version when status == STATUS_VERSION
uint8 target_y   # intended target y (5 cm / unit)
uint8 seq        # heartbeat counter, increments each send (liveness + loss detection)
