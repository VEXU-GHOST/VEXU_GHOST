# V5 SERIAL NODE
## Imports
### Own header
ghost_v5/serial/v5_serial_node
### Globals 
ghost_v5/globals/v5_globals
### Motor
ghost_v5/motor/v5_motor_interface
### Devices
ghost_v5_interfaces/devices/joystick_device_interface
ghost_v5_interfaces/devices/motor_device_interface
ghost_v5_interfaces/devices/rotation_sensor_device_interface
### Pros
pros/adi
pros/apix
pros/misc

## Namespace
ghost_v5

## Classes
V5SerialNode::V5SerialNode(std::shared_ptr<RobotHardwareInterface> robot_hardware_interface_ptr)
- Consumes a device config pointer
- Creates and configures serial interface
- 
V5SerialNode::~V5SerialNode() 
- Overload

###coprocessor CRUD operations for Actuator
void V5SerialNode::initSerial()
bool V5SerialNode::readV5ActuatorUpdate()
void V5SerialNode::updateActuatorCommands(std::vector<unsigned char> & buffer)
void V5SerialNode::writeV5StateUpdate()


# IMPORTANT PIPELINE FOR INTER ROBOT LINK
```
// Relays the inter-robot comms payload between the hardware interface and the VEXlink radio. The V5
// brain is a transparent byte pipe: it never interprets the payload (the field layout lives in ROS).
// Uses the packeted transmit/receive API so VEXlink handles framing + checksum for us (no COBS, no
// disallowed bytes on the radio hop).
void relay_inter_robot_link()
{
  auto & link = v5_globals::inter_robot_link;
  auto & rhi = v5_globals::robot_hardware_interface_ptr;

  // Report the VEXlink radio status up to the coprocessor (published in V5SensorUpdate). Set it even
  // when the link is down so a dropped link reads as link_connected == false on the ROS side.
  const bool link_connected = link && link->connected();
  rhi->setInterRobotLinkConnected(link_connected);
  if (!link_connected) {
    return;
  }

  // Receive: pull the peer's most recent packet (if a full one is buffered) into the inbound slot.
  // The next writeV5StateUpdate() serializes it out to the coprocessor.
  if (link->raw_receivable_size() >= ghost_v5_interfaces::inter_robot::OTHER_ROBOT_PACKET_SIZE) {
    ghost_v5_interfaces::inter_robot::OtherRobotBytes rx{};
    uint32_t received = link->receive(rx.data(), rx.size());
    if (received == rx.size()) {
      rhi->setInterRobotRx(rx);
    }
  }

  // Transmit: relay this robot's outbound payload, throttled to the VEXlink data rate. EBUSY/PROS_ERR
  // (no FIFO room) simply skips this cycle and retries next — never blocks the 10 ms loop.
  uint32_t now = pros::millis();
  if (now - v5_globals::last_inter_robot_link_tx >= v5_globals::inter_robot_link_tx_period_ms) {
    auto tx = rhi->getInterRobotTx();
    uint32_t result = link->transmit(tx.data(), tx.size());
    if (result == tx.size()) {
      v5_globals::last_inter_robot_link_tx = now;
    }
  }
}
```