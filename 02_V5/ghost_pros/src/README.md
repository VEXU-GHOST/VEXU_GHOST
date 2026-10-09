# V5 GHOST PROS

## Classes
void calibrateIMU(std::shared_ptr<pros::Imu> imu_ptr)
void exit_main_loop(const std::exception & e)
void zero_actuators()
void update_actuators()
void screen_update_loop()
void actuator_timeout_loop()
void reader_loop()
void relay_inter_robot_link()
void ghost_main_loop()
void initialize()
void disabled()
void competition_initialize()
void autonomous()
void opcontrol()

## Flow of Inter-Robot Connection
// Inter-Robot Comms slot (opaque bytes relayed over VEXlink). The V5 brain emits the payload it
  // received from the peer (rx) on the sensor-update stream; the coprocessor emits the payload to
  // transmit (tx) on the actuator-command stream.
  const auto & inter_robot_slot =
    (hardware_type_ == hardware_type_e::V5_BRAIN) ? inter_robot_rx_ : inter_robot_tx_;
  serial_data.insert(serial_data.end(), inter_robot_slot.begin(), inter_robot_slot.end());