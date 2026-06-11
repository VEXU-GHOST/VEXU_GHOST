ghost_msgs/V5DeviceHeader device_header

float32 rx_x      # partner robot x position (meters, map frame)
float32 rx_y      # partner robot y position (meters, map frame)
float32 rx_theta  # partner robot heading (radians, map frame, CCW positive)
bool    rx_valid  # true once a pose packet has been received from partner
