#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

using namespace std::chrono_literals;

class ROSPublisher : public rclcpp::Node {
public:
    ROSPublisher() :
        Node("marker_publisher"),
        count_(0),
        path_index_(0) {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ROSPublisher::timer_callback, this));
        
        // Generate poses along the straight line
        for (double x = 0.0; x <= 5; x += 0.1) {
            double y = 0.0; // Assuming the line lies in the xy-plane
            double z = 0.0;
            path_.push_back(createPose(x, y, z));
        }
    }

private:
    void timer_callback() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        auto next_pose = path_[path_index_];
        marker.pose.position.x = next_pose.position.x;
        marker.pose.position.y = next_pose.position.y;
        marker.pose.position.z = next_pose.position.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        publisher_->publish(marker);

        path_index_ = (path_index_ + 1) % path_.size();
    }

    geometry_msgs::msg::Pose createPose(double x, double y, double z) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
    std::vector<geometry_msgs::msg::Pose> path_;
    size_t path_index_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROSPublisher>());
    rclcpp::shutdown();
    return 0;
}
