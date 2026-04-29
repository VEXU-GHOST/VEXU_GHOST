#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "VL53L4CD.h"
#include "I2C_interfacing.h"

namespace ghost_sensing
{

class PublisherNode : public rclcpp::Node
{
  std::shared_ptr<ghost_sensing::VL53L4CD> color_sensor;

public:

private:
  void timer_callback();

  int main(int argc, char * argv[]);


};

}
