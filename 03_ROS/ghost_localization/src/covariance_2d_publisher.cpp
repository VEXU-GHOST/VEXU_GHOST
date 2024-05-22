#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ghost_localization
{

class Covariance2DPublisher : public rclcpp::Node
{
public:
  Covariance2DPublisher()
  : rclcpp::Node("covariance_2d_publisher")
  {
    declare_parameter("input_topic_names", std::vector<std::string>{});
    std::vector<std::string> input_topic_names =
      get_parameter("input_topic_names").as_string_array();

    if (input_topic_names.size() == 0) {
      throw std::runtime_error("[Covariance2DPublisher] Error: input_topic_names cannot be empty!");
    }

    for (const auto & name : input_topic_names) {
      std::cout << "Creating publisher for topic: " << name << "/pose_2d" << std::endl;
      std::cout << "Creating subscriber for topic: " << name << std::endl;
      auto pub = create_publisher<nav_msgs::msg::Odometry>(name + "/pose_2d", 10);
      m_output_pubs.push_back(pub);
      m_input_subs.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          name, 10, [pub](const nav_msgs::msg::Odometry::SharedPtr msg) {
            nav_msgs::msg::Odometry msg_2d{};
            msg_2d.header = msg->header;
            msg_2d.child_frame_id = msg->child_frame_id;
            msg_2d.pose.pose = msg->pose.pose;
            msg_2d.twist.twist = msg->twist.twist;

            msg_2d.pose.covariance[0] = msg->pose.covariance[0];
            msg_2d.pose.covariance[1] = msg->pose.covariance[1];
            msg_2d.pose.covariance[5] = msg->pose.covariance[5];
            msg_2d.pose.covariance[6] = msg->pose.covariance[6];
            msg_2d.pose.covariance[7] = msg->pose.covariance[7];
            msg_2d.pose.covariance[11] = msg->pose.covariance[11];
            msg_2d.pose.covariance[30] = msg->pose.covariance[30];
            msg_2d.pose.covariance[31] = msg->pose.covariance[31];
            msg_2d.pose.covariance[35] = msg->pose.covariance[35];

            msg_2d.twist.covariance[0] = msg->twist.covariance[0];
            msg_2d.twist.covariance[1] = msg->twist.covariance[1];
            msg_2d.twist.covariance[5] = msg->twist.covariance[5];
            msg_2d.twist.covariance[6] = msg->twist.covariance[6];
            msg_2d.twist.covariance[7] = msg->twist.covariance[7];
            msg_2d.twist.covariance[11] = msg->twist.covariance[11];
            msg_2d.twist.covariance[30] = msg->twist.covariance[30];
            msg_2d.twist.covariance[31] = msg->twist.covariance[31];
            msg_2d.twist.covariance[35] = msg->twist.covariance[35];

            pub->publish(msg_2d);
          }));
    }
  }

private:
  std::vector<std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>>> m_input_subs;
  std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>> m_output_pubs;
};

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_localization::Covariance2DPublisher>());
  rclcpp::shutdown();
  return 0;
}
