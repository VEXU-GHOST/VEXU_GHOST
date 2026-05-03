
#include <spawn.h>
#include "ghost_msgs/srv/start_recorder.hpp"
#include "ghost_msgs/srv/stop_recorder.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"
#include "ghost_msgs/msg/states/v5_competition.hpp"


// basic idea 
// once recording stopped - is_connected is false for 5 seconds or more- in bag_recorder_service.cpp
// replay recording to find is_autonomous is true and cut out everything from bag before 
//