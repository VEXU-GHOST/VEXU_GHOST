ghost_msgs/V5DeviceHeader device_header

int32 position_command
float32 velocity_command
float32 torque_command
float32 voltage_command
int32 current_limit

bool position_control
bool velocity_control
bool torque_control
bool voltage_control

# Currently unsupported
bool activate_braking
string brake_mode
bool set_zero_position
int32 zero_position
