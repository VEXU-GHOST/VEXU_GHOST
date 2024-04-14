#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include"geometry_msgs/msg/pose.hpp"
#include <map>
#include <vector>
#include<iostream>

using namespace std::chrono_literals;

class AnimationPublisher : public rclcpp::Node
{

  // std::string mode_ = this->get_parameter("mode").as_string();
//  this->declare_parameters_from_file("/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/config/test.yaml");
// struct Point{
//   double x;
//   double y;
// };

  auto parameters = this->get_parameters({"mode_playback", "mode_visualization"});
  mode_playback_ = parameters["mode_playback"].as_bool();
  mode_visualization_ = parameters["mode_visualization"].as_bool();

  AnimationPublisher():Node("marker_publisher"), count_(0), path_index_(0){

      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);

      // mode_service_ = this->create_service<std_srvs::srv::SetBool>(
      // "switch_trajectory_mode", std::bind(&AnimationPublisher::modeSwitchCallback, this,
      // std::placeholders::_1, std::placeholders::_2));

    //  mode_ = "playback"; //hardcoding mode to be playback omg

    timer_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::timer_callback, this));

    std::map<int64_t, std::vector<double>> mapOfPos;

    // Insert some elements into the map
    mapOfPos[1] = {0.0, 0.0, 0.0};
    mapOfPos[2] = {1.0, 0.0,0.0};
    mapOfPos[3] = {2.0, 0.0, 0.0};
    mapOfPos[4] = {3.0, 0.0, 0.0};
    mapOfPos[5] = {4.0, 0.0,0.0};
    mapOfPos[6] = {5.0, 0.0, 0.0};
    
// Iterating through the map and creating Pose objects
    for (const auto& pair : mapOfPos) {
        double x = pair.second[0];  // Accessing the first element of the vector
        double y = pair.second[1];  // Accessing the second element of the vector
        double z = pair.second[2];  // Accessing the third element of the vector
        path_.push_back(createPose(x, y, z));
}
  
    }

  private:


    void timer_callback()
  {

      auto marker_msg_ = visualization_msgs::msg::Marker();
      marker_msg_.header.frame_id = "map";
      marker_msg_.header.stamp = this->get_clock()->now();
      // marker_msg_.header.stamp = this->now();
      marker_msg_.ns = "basic_shapes";
      marker_msg_.id = 0;
      marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
      marker_msg_.action = visualization_msgs::msg::Marker::ADD;

      auto next_pose = path_[path_index_];
      marker_msg_.pose.position.x = next_pose.position.x;
      marker_msg_.pose.position.y = next_pose.position.y;
      marker_msg_.pose.position.z = next_pose.position.z;
      marker_msg_.pose.orientation.x = 0.0;
      marker_msg_.pose.orientation.y = 0.0;
      marker_msg_.pose.orientation.z = 0.0;
      marker_msg_.pose.orientation.w = 1.0;
      marker_msg_.scale.x = 1.0;
      marker_msg_.scale.y = 1.0;
      marker_msg_.scale.z = 1.0;
      marker_msg_.color.a = 1.0; // Alpha
      marker_msg_.color.r = 1.0; // Red
      marker_msg_.color.g = 0.0; // Green
      marker_msg_.color.b = 0.0; // Blue

      
      publisher_->publish(marker_msg_);
      path_index_ = (path_index_ + 1) % path_.size();


  //     // if (mode_ == "playback"){
        
  //     //   }
  //     // else if(mode_ == "visualization"){

  //     //  } 

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
    // std::string mode_;


};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnimationPublisher>());
  rclcpp::shutdown();
  return 0;
}
