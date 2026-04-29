#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "I2C_interfacing.h"
#include "Sensor_Node2.h"

class PublisherNode_Distance : public rclcpp::Node
{
public:
  std::shared_ptr<ghost_sensing::VL53L4CD> distance_sensor;

  PublisherNode_Distance()
  : Node("Distance_Sensor_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("Distance", 10);     //stores 10 readings for averaging idk

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PublisherNode_Distance::timer_callback, this));


    this->declare_parameter<int>("address", 0x41);            // example
    this->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");

    const int address = this->get_parameter("address").as_int();
    const auto dev = this->get_parameter("i2c_device").as_string();

    auto iface = std::make_shared<ghost_sensing::I2C_interfacing>(
      "/dev/i2c-1",
      this->get_logger());


    distance_sensor = std::make_shared<ghost_sensing::VL53L4CD>(iface, address);


    if (!distance_sensor->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize VL53L4CD sensor");
      //pritn debug statement
    }
    if (!distance_sensor->VL53L4CD_StartRanging()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start VL53L4CD sensor");
      //pritn debug statement
    }

    RCLCPP_INFO(this->get_logger(), "VL53L4CD sensor initialized successfully");
  }

private:
  void timer_callback()
  {
    ghost_sensing::VL53L4CD::VL53L4CD_ResultsData_t data;
    if (distance_sensor->VL53L4CD_GetResult(&data) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read sensor data");
      return;
    }
    
    // Clear the interrupt so the sensor can take the next reading
    distance_sensor->VL53L4CD_ClearInterrupt();

    auto message = std_msgs::msg::String();
    message.data = "Dist: " + std::to_string(data.distance_mm) + " mm (Status: " + std::to_string(data.range_status) + ")";

    RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PublisherNode_Distance>();

  //ghost_sensing::VL53L4CD::init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  node->distance_sensor->VL53L4CD_StopRanging();
  return 0;
}
