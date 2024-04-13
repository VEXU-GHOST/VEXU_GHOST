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

  std::string mode_ = this->get_parameter("mode").as_string();\
//  this->declare_parameters_from_file("/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/config/test.yaml");
// struct Point{
//   double x;
//   double y;
// };

  public:
    AnimationPublisher()
    : Node("animation_publisher"), count_(0),path_index_(0)
    {
      publisher1_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);
      publisher2_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);

      // mode_service_ = this->create_service<std_srvs::srv::SetBool>(
      // "switch_trajectory_mode", std::bind(&AnimationPublisher::modeSwitchCallback, this,
      // std::placeholders::_1, std::placeholders::_2));

     mode_ = "playback"; //hardcoding mode to be playback omg

      timer_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::timer_callback, this));

      // x_data_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
      // y_data_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
      // time_vector_ = std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

        //     for (double x = 0.0; x <= 5; x += 0.1) {
        //     double y = 0.0; // Assuming the line lies in the xy-plane
        //     double z = 0.0;
        //     path_.push_back(createPose(x, y, z));
        // }
  
    }

  private:


  //   void timer_callback()
  // {

  //     auto marker_msg_ = visualization_msgs::msg::Marker();
  //     marker_msg_.header.frame_id = "map";
  //     marker_msg_.header.stamp = this->get_clock()->now();
  //     // marker_msg_.header.stamp = this->now();
  //     marker_msg_.ns = "basic_shapes";
  //     marker_msg_.id = 0;
  //     marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
  //     marker_msg_.action = visualization_msgs::msg::Marker::ADD;

  //     auto next_pose = path_[path_index_];
  //     marker_msg_.pose.position.x = next_pose.position.x;
  //     marker_msg_.pose.position.y = next_pose.position.y;
  //     marker_msg_.pose.position.z = next_pose.position.z;
  //     marker_msg_.pose.orientation.x = 0.0;
  //     marker_msg_.pose.orientation.y = 0.0;
  //     marker_msg_.pose.orientation.z = 0.0;
  //     marker_msg_.pose.orientation.w = 1.0;
  //     marker_msg_.scale.x = 1.0;
  //     marker_msg_.scale.y = 1.0;
  //     marker_msg_.scale.z = 1.0;
  //     marker_msg_.color.a = 1.0; // Alpha
  //     marker_msg_.color.r = 1.0; // Red
  //     marker_msg_.color.g = 0.0; // Green
  //     marker_msg_.color.b = 0.0; // Blue

      
  //     publisher_->publish(marker_msg_);
  //     path_index_ = (path_index_ + 1) % path_.size();


  //     // if (mode_ == "playback"){
        
  //     //   }
  //     // else if(mode_ == "visualization"){

  //     //  } 

  //       }

  //     geometry_msgs::msg::Pose createPose(double x, double y, double z) {
  //     geometry_msgs::msg::Pose pose;
  //       pose.position.x = x;
  //       pose.position.y = y;
  //       pose.position.z = z;
  //       return pose;

  //     }


  void timer_callback(){
    auto message = visualization_msgs::msg::Marker();
    
     message_.header.frame_id = "map";
     message.header.stamp = this->get_clock()->now();
      // marker_msg_.header.stamp = this->now();
      message.ns = "basic_shapes";
      message.id = 0;
      message.type = visualization_msgs::msg::Marker::SPHERE;
      message.action = visualization_msgs::msg::Marker::ADD;

      message.pose.position.x = next_pose.position.x;
      message.pose.position.y = next_pose.position.y;
      message.pose.position.z = next_pose.position.z;
      message.pose.orientation.x = 0.0;
      message.pose.orientation.y = 0.0;
      message.pose.orientation.z = 0.0;
      message.pose.orientation.w = 1.0;
      message.scale.x = 1.0;
      message.scale.y = 1.0;
      message.scale.z = 1.0;
      messagecolor.a = 1.0; // Alpha
      messagecolor.r = 1.0; // Red
      message.color.g = 0.0; // Green
      message.color.b = 0.0; // Blue

    std::map<std::string, std::vector<int>> dataMap;

    // Example data
    std::vector<int> values1 = {1, 2, 3};
    std::vector<int> values2 = {4, 5, 6};
    std::vector<int> values3 = {7, 8, 9};

    // Insert elements into the map
    dataMap["John"] = values1;
    dataMap["Alice"] = values2;
    dataMap["Bob"] = values3;


     for (auto& pair : dataMap) {
        // message.id = names.push_back(pair.first);
        // sensor_msgs::msg::Int32MultiArray array_msg;
        message. = pair.second;
        message.values->values.push_back(array_msg);
    }

    // Publish the message
    publisher_->publish(*message);

  }

  


      
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

  


    // void  modeSwitchCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    // // auto response = std_srvs::srv::SetBool::Response();
    // if (request->data) {
    //     mode_ = "visualization";
    // } else {
    //     mode_ = "playback";
    // }
    // response->success = true;
    // response->message = "Mode switched successfully";

    // // publisher_->publish(response);

    // }

// void modeSwitchCallback(){
//   if (mode_ == "playback"){
//     //random code here
//   }
//   else if(mode_ == "visualization"){
//     //more random code here
//   }
// }


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

#hi