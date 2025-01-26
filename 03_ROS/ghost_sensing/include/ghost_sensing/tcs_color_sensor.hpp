#include <rclcpp/rclcpp.hpp>
#include <driver_tcs34725_interface.h>

namespace ghost_sensing {
class TCSColorSensorNode : public rclcpp::Node
{
public:
  TCSColorSensorNode();

  void start();

};

}