#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
// #include <ghost_msgs/msg/labeled_vector.hpp>
// #include <ghost_msgs/msg/labeled_vector_map.hpp>
#include "rclcpp/rclcpp.hpp"
#include <vector>

using namespace std::chrono_literals;

class ROSPublisher : public rclcpp::Node
{
public:
    ROSPublisher() : Node("marker_publisher"),
                     count_(0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ROSPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        for (int i = 0; i < 10; i++)
        {
            visualization_msgs::msg::Marker marker;
            createSphereMarker(i, marker); // Create and fill marker directly
            marker_array_msg->markers.push_back(marker);
        }
        publisher_->publish(*marker_array_msg);
    }

    void createSphereMarker(int id, visualization_msgs::msg::Marker &marker)
    {
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Define a function of time for position calculation
        //auto t = this->get_clock()->now(); // Current time in seconds
                                           // Current time in seconds
        double x = 0;                      // Example function: x = t * cos(t + id)
        double y = 0;                      // Example function: y = t * sin(t + id)
        double z = 0;                      // Example function: z = t

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROSPublisher>());
    rclcpp::shutdown();
    return 0;
}
