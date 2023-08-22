string motor_name
int32 device_id             # Corresponds to V5 Port Number

int32 desired_position         # Degrees
float32 desired_velocity    # RPM
float32 desired_torque      # N-m
float32 desired_voltage     # Normalized (-1.0 -> 1.0)
int32 current_limit         # milliAmps

bool position_control
bool velocity_control
bool torque_control
bool voltage_control