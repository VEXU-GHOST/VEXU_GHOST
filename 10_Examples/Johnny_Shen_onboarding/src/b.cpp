#include "Johnny_Shen_onboarding/b.hpp"
using std::placeholders::_1;

Subscribertest::Subscribertest()
: rclcpp::Node("hello1_node")
{
  robot_sub_ = create_subscription<std_msgs::msg::String>(
    "/hello",
    10,
    std::bind(&Subscribertest::printer,this,_1));

}
void Subscribertest::printer(std_msgs::msg::String::SharedPtr msg){
    std::cout << msg->data;
// RCLCPP_INFO(this, msg->data);
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Subscribertest>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}