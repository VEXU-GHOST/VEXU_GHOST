#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class AnimationPublisher : public rclcpp::Node {
	// declare_parameters_from_file("/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/config/test.yaml");
	// declare_parameter("mode", "default");
	// std::string test_param = get_parameter("mode").as_string()
	// std::string mode_ = this->get_parameter("mode").as_string();
	// auto node = std::make_shared<rclcpp::Node>("parameter_loader");

<<<<<<< HEAD
 	// loadParametersFromFile("/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/config/test.yaml");
	// this->declare_parameter<std::string>("mode", "playback");
	// mode_ = this->get_parameter("mode").as_string();

public:
	AnimationPublisher() :
		Node("marker_publisher"),
		count_(0),
		path_index_(0){
		std::map<double, std::vector<double> > mapOfPos;
=======
	// loadParametersFromFile("/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/config/test.yaml");
	// this->declare_parameter<std::string>("mode", "playback");
	// mode_ = this->get_parameter("mode").as_string();

public:
	AnimationPublisher() :
		Node("marker_publisher"),
		count_(0),
		path_index_(0){
		std::map<double, std::vector<double> > mapOfPos;
>>>>>>> 31c97f91329df1e15cf447daf160c8c4a164f002

		this->declare_parameter<std::string>("yaml_path", "");

		// Get the YAML file path parameter
		yaml_path_ = this->get_parameter("yaml_path").as_string();

		// // Load parameters from the YAML file
		// this->load_parameters_from_file(yaml_path_);

<<<<<<< HEAD
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
=======
		// Get other parameters from the loaded parameters
		this->declare_parameter("mode", "visualization");
		mode_ = this->get_parameter("mode").as_string();
		// this->get_parameter("mode", mode_);
		std::cout << mode_ << std::endl;
>>>>>>> 31c97f91329df1e15cf447daf160c8c4a164f002


		// this->declare_parameter("mode", "default");
		// mode_ = this->get_parameter("mode").as_string();

		// std::string mode_ = "visualization";
		publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);
		// Insert some elements into the map

		mapOfPos[1.0] = {0.0, 0.0, 0.0};
		mapOfPos[2.0] = {1.0, 0.0,0.0};
		mapOfPos[3.0] = {2.0, 0.0, 0.0};
		mapOfPos[4.0] = {3.0, 0.0, 0.0};
		mapOfPos[5.0] = {4.0, 0.0,0.0};
		mapOfPos[6.0] = {5.0, 0.0, 0.0};
		mapOfPos[7.0] = {6.0, 0.0, 0.0};
		mapOfPos[8.0] = {7.0, 0.0,0.0};
		mapOfPos[9.0] = {8.0, 0.0, 0.0};
		mapOfPos[10.0] = {9.0, 0.0, 0.0};
		mapOfPos[11.0] = {10.0, 0.0,0.0};
		mapOfPos[12.0] = {11.0, 0.0, 0.0};

		// timer_ = this->create_wall_timer(10ms, std::bind(&AnimationPublisher::printMessage, this, mode_));


		if(mode_ == "playback"){
			timer_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::playback_callback, this));
			// poseObjects(linearInterpolation3D(0.1, mapOfPos), path_)
			for(const auto& pair : linearInterpolation3D(0.1, mapOfPos)){
				double x = pair.second[0];  // Accessing the first element of the vector
				double y = pair.second[1];  // Accessing the second element of the vector
				double z = pair.second[2];  // Accessing the third element of the vector
				// int id = pair.first;
				path_.push_back(createPose(x, y, z));
			}
		}

		else if(mode_ == "visualization"){
			timer1_ = this->create_wall_timer(500ms, std::bind(&AnimationPublisher::visualization_callback, this));

			for(const auto& pair : linearInterpolation3D(0.5, mapOfPos)){
				double x = pair.second[0];  // Accessing the first element of the vector
				double y = pair.second[1];  // Accessing the second element of the vector
				double z = pair.second[2];  // Accessing the third element of the vector
				// int id = pair.first;
				path_.push_back(createPose(x, y, z));
			}
		}
	}

<<<<<<< HEAD
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
=======
	// Iterating through the map and creating Pose objects
	// for(const auto& pair : linearInterpolation3D(0.1, mapOfPos)){
	// 	double x = pair.second[0];  // Accessing the first element of the vector
	// 	double y = pair.second[1];  // Accessing the second element of the vector
	// 	double z = pair.second[2];  // Accessing the third element of the vector
	// 	// int id = pair.first;
	// 	path_.push_back(createPose(x, y, z));


private:


	// void load_parameters_from_file(const std::string& yaml_path) {
	//     // Open the YAML file
	//     YAML::Node config = YAML::LoadFile(yaml_path);

	//     // Assuming the YAML file contains parameters under a 'parameters' key
	//     if (config["parameters"]) {
	//         // Access parameters and process them accordingly
	//         YAML::Node parameters = config["parameters"];

	//         // Example: Print out each parameter
	//         for (const auto& param : parameters) {
	//             std::string name = param.first.as<std::string>();
	//             std::string value = param.second.as<std::string>();
	//             std::cout << "Parameter: " << name << ", Value: " << value << std::endl;
	//         }
	//     } else {
	//         // Handle the case where the 'parameters' key is not found
	//         std::cerr << "Error: 'parameters' key not found in YAML file." << std::endl;
	//     }
	// }

	// void printMessage(std::string& message) {
	// 	std::cout << message << std::endl; // Print to standard output
	// }

	void playback_callback(){
		auto marker_msg_ = visualization_msgs::msg::Marker();
		marker_msg_.header.frame_id = "map";
		marker_msg_.header.stamp = this->get_clock()->now();
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
	}


	// void visualization_callback(){
	// 	//
	// 	// visualization_msgs::msg::MarkerArray marker_ar;
	// 	// auto marker_msg_ = visualization_msgs::msg::MarkerArray();


	// 	// for(int i = 0; i < 7; ++i){
	// 		auto marker_msg_ = visualization_msgs::msg::Marker();
	// 		marker_msg_.header.frame_id = "map";
	// 		marker_msg_.header.stamp = this->get_clock()->now();
	// 		marker_msg_.ns = "basic_shapes";
	// 		marker_msg_.id = i;
	// 		marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
	// 		marker_msg_.action = visualization_msgs::msg::Marker::ADD;
	// 		auto next_pose = path_[path_index_];
	// 		marker_msg_.pose.position.x = next_pose.position.x;
	// 		marker_msg_.pose.position.y = next_pose.position.y;
	// 		marker_msg_.pose.position.z = next_pose.position.z;
	// 		marker_msg_.pose.orientation.x = 0.0;
	// 		marker_msg_.pose.orientation.y = 0.0;
	// 		marker_msg_.pose.orientation.z = 0.0;
	// 		marker_msg_.pose.orientation.w = 1.0;
	// 		marker_msg_.scale.x = 1.0;
	// 		marker_msg_.scale.y = 1.0;
	// 		marker_msg_.scale.z = 1.0;
	// 		marker_msg_.color.a = 1.0;// Alpha
	// 		marker_msg_.color.r = 1.0; // Red
	// 		marker_msg_.color.g = .5 * i; // Green
	// 		marker_msg_.color.b = 0.0; // Blue
	// 		// Add the marker to the marker array
	// 		// Move to the next pose in the path
	// 		publisher_->publish(marker_msg_);
	// 		path_index_ = (path_index_ + 1) % path_.size();
	// 	}

	void visualization_callback(){
		auto marker_msg_ = visualization_msgs::msg::Marker();

		marker_msg_.header.frame_id = "map";
		marker_msg_.header.stamp = this->get_clock()->now();
		marker_msg_.ns = "basic_shapes";
		marker_msg_.id = 1;
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
		marker_msg_.color.a = 1.0;// Alpha
		marker_msg_.color.r = 1.0; // Red
		marker_msg_.color.g = 0.0; // Green
		marker_msg_.color.b = 0.0; // Blue
		// Add the marker to the marker array
		// Move to the next pose in the path
		publisher_->publish(marker_msg_);
		path_index_ = (path_index_ + 1) % path_.size();
		// Publish the marker array

		marker_msg_.header.frame_id = "map";
		marker_msg_.header.stamp = this->get_clock()->now();
		marker_msg_.ns = "basic_shapes";
		marker_msg_.id = 2;
		marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
		marker_msg_.action = visualization_msgs::msg::Marker::ADD;
		marker_msg_.pose.position.x = next_pose.position.x + .5;
		marker_msg_.pose.position.y = next_pose.position.y;
		marker_msg_.pose.position.z = next_pose.position.z;
		marker_msg_.pose.orientation.x = 0.0;
		marker_msg_.pose.orientation.y = 0.0;
		marker_msg_.pose.orientation.z = 0.0;
		marker_msg_.pose.orientation.w = 1.0;
		marker_msg_.scale.x = 1.0;
		marker_msg_.scale.y = 1.0;
		marker_msg_.scale.z = 1.0;
		marker_msg_.color.a = .7;// Alpha
		marker_msg_.color.r = 1.0; // Red
		marker_msg_.color.g = 0.5; // Green
		marker_msg_.color.b = 0.0; // Blue
		// Add the marker to the marker array
		// Move to the next pose in the path
		publisher_->publish(marker_msg_);
		path_index_ = (path_index_ + 1) % path_.size();

		marker_msg_.header.frame_id = "map";
		marker_msg_.header.stamp = this->get_clock()->now();
		marker_msg_.ns = "basic_shapes";
		marker_msg_.id = 3;
		marker_msg_.type = visualization_msgs::msg::Marker::SPHERE;
		marker_msg_.action = visualization_msgs::msg::Marker::ADD;
		marker_msg_.pose.position.x = next_pose.position.x + 1;
		marker_msg_.pose.position.y = next_pose.position.y;
		marker_msg_.pose.position.z = next_pose.position.z;
		marker_msg_.pose.orientation.x = 0.0;
		marker_msg_.pose.orientation.y = 0.0;
		marker_msg_.pose.orientation.z = 0.0;
		marker_msg_.pose.orientation.w = 1.0;
		marker_msg_.scale.x = 1.0;
		marker_msg_.scale.y = 1.0;
		marker_msg_.scale.z = 1.0;
		marker_msg_.color.a = .5;// Alpha
		marker_msg_.color.r = 1.0; // Red
		marker_msg_.color.g = 0.5; // Green
		marker_msg_.color.b = 0.5; // Blue
		// Add the marker to the marker array
		// Move to the next pose in the path
		publisher_->publish(marker_msg_);
		path_index_ = (path_index_ + 1) % path_.size();
	}
	geometry_msgs::msg::Pose createPose(double x, double y, double z) {
		geometry_msgs::msg::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		return pose;
	}


	std::map<double, std::vector<double> > linearInterpolation3D(double t, const std::map<double, std::vector<double> >& map) {
		// Clamp interpolation parameter t between [0, 1]
		t = std::max(0.0, std::min(1.0, t));

		std::map<double, std::vector<double> > combined_map;

		// Iterate over the map to interpolate values
		auto it = map.begin();
		while(it != map.end()){
			// Get the current key and value
			double current_key = it->first;
			const std::vector<double>& current_value = it->second;

			// Find the next element
			auto next_it = std::next(it);
			if(next_it == map.end()){
				// No more elements to interpolate
				break;
			}

			// Get the next key and value
			double next_key = next_it->first;
			const std::vector<double>& next_value = next_it->second;

			// Perform linear interpolation between current and next values
			std::vector<double> interpolated_value;
			for(size_t i = 0; i < current_value.size(); ++i){
				double interpolated_component = current_value[i] + t * (next_value[i] - current_value[i]);
				interpolated_value.push_back(interpolated_component);
			}

			auto key = (current_key + next_key) / 2;

			// Insert the original and interpolated values into the combined map
			combined_map[current_key] = current_value;
			combined_map[key] = interpolated_value;

			// Move to the next element
			++it;
		}

		return combined_map;
	}

	void poseObjects(std::map<double, std::vector<double> >& map, std::vector<geometry_msgs::msg::Pose> path){
		for(const auto& pair : map){
			double x = pair.second[0];  // Accessing the first element of the vector
			double y = pair.second[1];  // Accessing the second element of the vector
			double z = pair.second[2];  // Accessing the third element of the vector
			// int id = pair.first;
			path.push_back(createPose(x, y, z));
		}
	}
>>>>>>> 31c97f91329df1e15cf447daf160c8c4a164f002


	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer1_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
	size_t count_;
	std::vector<geometry_msgs::msg::Pose> path_;
	size_t path_index_;
	std::string mode_;
	std::string yaml_path_;
};
<<<<<<< HEAD
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnimationPublisher>());
  rclcpp::shutdown();
  return 0;
}
=======


int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AnimationPublisher>());
	rclcpp::shutdown();
	return 0;
}
>>>>>>> 31c97f91329df1e15cf447daf160c8c4a164f002
