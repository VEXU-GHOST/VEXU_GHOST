#include "rviz_animators/subscriber.hpp"

namespace rviz
{

    ROSSubscriber::ROSSubscriber() : Node("subscribers")
    {

        this->declare_parameter("test_param", "/estimation/pose");
        std::string test_param = this->get_parameter("test_param").as_string();

        std::cout << test_param << std::endl;

        // Create a subscription to the "marker_topic" topic with a queue size of 10
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "marker_topic", 10, std::bind(&ROSSubscriber::topic_callback, this, std::placeholders::_1));

        // vector_subscription_ = this->create_subscription<ghost_msgs::msg::LabeledVectorMap>(
        //     "vector_topic", 10, std::bind(&ROSSubscriber::vector_topic_callback, this, std::placeholders::_1));
        // Uncomment the following lines if you want to subscribe to a trajectory topic
        // trajectory_subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        //     "trajectory_topic", 10, std::bind(&ROSSubscriber::trajectory_topic_callback, this, std::placeholders::_1));
    }

    void ROSSubscriber::topic_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        auto diff = now - msg->markers[0].header.stamp;
        // Log the X-coordinate of the first marker in the array
        RCLCPP_INFO(this->get_logger(), "X-coord is: %f", msg->markers[0].pose.position.x);
    }

    // void ROSSubscriber::vector_topic_callback(const ghost_msgs::msg::LabeledVectorMap::SharedPtr msg)
    // {
    //     // plays message back in rviz
    //     // auto now = this->get_clock()->now();
    //     // auto diff = now - msg->markers[0].header.stamp;
    //     // Log the X-coordinate of the first marker in the array
    //     RCLCPP_INFO(this->get_logger(), "X-coord is: %f", msg->markers[0].pose.position.x);
   // }
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
   // rclcpp::Subscription<ghost_msgs::msg::LabeledVector>::SharedPtr vector_subscription_;

    // Uncomment the following function if you want to handle messages of type trajectory_msgs::msg::JointTrajectoryPoint
    // void ROSSubscriber::topic_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
    //     auto now = this->get_clock()->now();
    //     auto diff = now - msg->header.stamp;
    //     // Log the X-coordinate of the received trajectory point
    //     RCLCPP_INFO(this->get_logger(), "X-coord is: %f", msg->positions[0]);
    // }

} // namespace rviz
