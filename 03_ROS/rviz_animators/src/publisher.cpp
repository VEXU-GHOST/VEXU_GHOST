#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>

using namespace std::chrono_literals;

class AnimationPublisher : public rclcpp::Node
{

// struct Point{
//   double x;
//   double y;
// };

// std::vector<Point> vector_;
std::vector<double> x_data_vector_;
std::vector<double> y_data_vector_;
std::vector<double> time_vector_;

  // visualization_msgs::msg::Marker marker_msg_;

  public:
    AnimationPublisher()
    : Node("animation_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);

      timer_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::timer_callback, this));

      x_data_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
      y_data_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
      time_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    }

  private:

   vector_values(int x_vector,int  y_vector, i){


    int x_vector_value = x_vector[i];
    int y_vector_value = y_vector[i];

    int array[2];
    array[0] = x_vector_value;
    array[1] = y_vector_value;

    return array;
  }


    void timer_callback()
  {

      auto marker_msg_ = visualization_msgs::msg::Marker();
      // marker_msg_.another_field = 0;


      // RCLCPP_INFO(this->get_logger(), "Publishing: ", marker_msg_);
      marker_msg_.header.frame_id = "map";
      marker_msg_.header.stamp = this->get_clock()->now();
      // marker_msg_.header.stamp = this->now();
      marker_msg_.ns = "basic_shapes";
      marker_msg_.id = 0;
      marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
      marker_msg_.action = visualization_msgs::msg::Marker::ADD;
      // marker_msg_.pose.position.x = 1;
      // marker_msg_.pose.position.y = 1;
      // marker_msg_.pose.position.z = 1;
      // marker_msg_.pose.orientation.x = 0.0;
      // marker_msg_.pose.orientation.y = 0.0;
      marker_msg_.pose.orientation.z = 0.0;
      marker_msg_.pose.orientation.w = 1.0;
      marker_msg_.scale.x = 1.0;
      marker_msg_.scale.y = 1.0;
      marker_msg_.scale.z = 1.0;
      marker_msg_.color.a = 1.0; // Alpha
      marker_msg_.color.r = 1.0; // Red
      marker_msg_.color.g = 0.0; // Green
      marker_msg_.color.b = 0.0; // Blue


      for(int i = 0; i < 10, i++){
      array = vector_values(x_data_vector_, y_data_vector_, i)
      marker_msg_.pose.position.x = array[0];
      marker_msg_.pose.position.y = array[1];


      }

      publisher_->publish(marker_msg_);
    
      
    // for (int i = 0; i < 10; ++i) {
    //     Point currentPos;
    //     currentPos.x = x*0.25;
    //     currentPos.y = y*0.25
    //     currentPos.z = 0;

    //     auto timestamp = std::chrono::system_clock::now(); // Record current timestamp
    //     vector.push_back({currentPos, timestamp}); // Store position and timestamp in vector
    //     Point.stamp = ros::Time::now() + ros::Duration(i);  // Example: increment timestamp by i seconds
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    // publisher_->publish(marker_msg_);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnimationPublisher>());
  rclcpp::shutdown();
  return 0;
}