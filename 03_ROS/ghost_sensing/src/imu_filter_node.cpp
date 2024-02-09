#include <iostream>
#include <ghost_sensing/imu_filter_node.hpp>

#include <ghost_util/vector_util.hpp>
#include <visualization_msgs/msg/marker.hpp>

using ghost_util::getRotationMatrixFromEulerAnglesDegrees;
using std::placeholders::_1;

namespace ghost_sensing {

IMUFilterNode::IMUFilterNode() :
	rclcpp::Node("imu_filter_node"),
	m_msg_count(0){
	// ROS Parameters
	declare_parameter("calculate_bias", false);
	m_calculate_bias = get_parameter("calculate_bias").as_bool();

	declare_parameter("sensor_frequency", 200.0);
	m_sensor_freq = get_parameter("sensor_frequency").as_double();

	declare_parameter("calibration_time", 20.0);
	m_calibration_time = get_parameter("calibration_time").as_double();

	declare_parameter("camera_roll", 0.0);
	auto roll = get_parameter("camera_roll").as_double();

	declare_parameter("camera_pitch", 0.0);
	auto pitch = get_parameter("camera_pitch").as_double();

	declare_parameter("camera_yaw", 0.0);
	auto yaw = get_parameter("camera_yaw").as_double();

	m_base_link_to_camera_rotation = getRotationMatrixFromEulerAnglesDegrees(roll, pitch, yaw);

	declare_parameter("camera_x", 0.0);
	auto camera_x = get_parameter("camera_x").as_double();

	declare_parameter("camera_y", 0.0);
	auto camera_y = get_parameter("camera_y").as_double();

	m_base_link_to_camera_translation = Eigen::Vector2d(camera_x, camera_y);

	// Handle Bias Calibration
	if(m_calculate_bias){
		m_num_msgs_init = (int) m_calibration_time * m_sensor_freq;
		m_imu_accel_samples.reserve(m_num_msgs_init);
		m_imu_gyro_samples.reserve(m_num_msgs_init);
		m_imu_accel_bias = Eigen::Vector3d(0.0, 0.0, 0.0);
		m_imu_gyro_bias = Eigen::Vector3d(0.0, 0.0, 0.0);
	}
	else{
		declare_parameter("accel_bias_x", 0.0);
		auto accel_bias_x = get_parameter("accel_bias_x").as_double();

		declare_parameter("accel_bias_y", 0.0);
		auto accel_bias_y = get_parameter("accel_bias_y").as_double();

		declare_parameter("accel_bias_z", 0.0);
		auto accel_bias_z = get_parameter("accel_bias_z").as_double();

		declare_parameter("gyro_bias_x", 0.0);
		auto gyro_bias_x = get_parameter("gyro_bias_x").as_double();

		declare_parameter("gyro_bias_y", 0.0);
		auto gyro_bias_y = get_parameter("gyro_bias_y").as_double();

		declare_parameter("gyro_bias_z", 0.0);
		auto gyro_bias_z = get_parameter("gyro_bias_z").as_double();

		m_imu_accel_bias = Eigen::Vector3d(accel_bias_x, accel_bias_y, accel_bias_z);
		m_imu_gyro_bias = Eigen::Vector3d(gyro_bias_x, gyro_bias_y, gyro_bias_z);
	}

	// ROS Topics
	m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
		"/camera/imu", 10, std::bind(&IMUFilterNode::imu_callback, this, _1));

	m_filtered_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
		"sensing/imu/filtered", 10);

	m_vel_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"viz/imu_gyro",
		10);

	m_accel_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"viz/imu_accel",
		10);

	m_vel_filtered_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"viz/imu_gyro_filtered",
		10);

	m_accel_filtered_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"viz/imu_accel_filtered",
		10);
}


void IMUFilterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
	auto raw_gyro_vector = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	auto raw_accel_vector = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

	// Estimate IMU Bias at initialization
	if(m_calculate_bias){
		if(m_msg_count < m_num_msgs_init){
			if(m_msg_count % 100 == 0){
				std::cout << "Initializing: " << m_msg_count / m_sensor_freq  << "s" << std::endl;
			}
			m_imu_accel_samples.push_back(raw_accel_vector);
			m_imu_gyro_samples.push_back(raw_gyro_vector);
			m_msg_count++;
		}
		else if(!m_calibration_complete){
			std::cout << "Calibrating ... " << std::endl;
			// After we complete initialization, IMU gyro bias

			auto calculateBiasVector =
				[&](const std::vector<Eigen::Vector3d>& samples, Eigen::Vector3d& bias){
					for(const auto & v : samples){
						bias += v;
					}
					bias /= samples.size();
				};
			auto calculateCovarianceMatrix =
				[&](const std::vector<Eigen::Vector3d>& samples, Eigen::Matrix3d& covariance){
				};

			calculateBiasVector(m_imu_accel_samples, m_imu_accel_bias);
			calculateBiasVector(m_imu_gyro_samples, m_imu_gyro_bias);

			m_calibration_complete = true;
		}
	}
	else if(!m_calibration_complete){
		std::cout << "Acceleration Bias Vector in Camera Frame:" << std::endl;
		std::cout << m_imu_accel_bias << std::endl;
		std::cout << "Gyro Bias Vector in Camera Frame:" << std::endl;
		std::cout << m_imu_gyro_bias << std::endl;
		m_calibration_complete = true;
	}

	// Remove Bias Terms
	auto filtered_accel_vector_base_link = m_base_link_to_camera_rotation * (raw_accel_vector - m_imu_accel_bias);
	auto filtered_gyro_vector_base_link = m_base_link_to_camera_rotation * (raw_gyro_vector - m_imu_gyro_bias);

	// Visualize
	const double gyro_scale = 1.0;
	const double accel_scale = 1.0;
	m_vel_viz_pub->publish(getMarkerArrayVector3(raw_gyro_vector, gyro_scale));
	m_vel_filtered_viz_pub->publish(getMarkerArrayVector3(filtered_gyro_vector_base_link, gyro_scale));
	m_accel_viz_pub->publish(getMarkerArrayVector3(raw_accel_vector, accel_scale));
	m_accel_filtered_viz_pub->publish(getMarkerArrayVector3(filtered_accel_vector_base_link, accel_scale));
}

visualization_msgs::msg::Marker IMUFilterNode::getDefaultArrowMsg(int id, builtin_interfaces::msg::Time stamp){
	visualization_msgs::msg::Marker msg{};
	msg.header.frame_id = "base_link";
	msg.header.stamp = stamp;
	msg.id = id;
	msg.action = 0;
	msg.type = 0;
	msg.scale.x = 0.05;
	msg.scale.y = msg.scale.x * 1.75;
	msg.scale.z = 0.075;
	msg.color.a = 1.0;
	return msg;
}

visualization_msgs::msg::MarkerArray IMUFilterNode::getMarkerArrayVector3(const Eigen::Vector3d& vector, double scale){
	// Marker Array Msg
	visualization_msgs::msg::MarkerArray viz_msg{};
	auto msg_time = this->get_clock()->now();
	int marker_id = 10;

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > arrow_configs{
		std::pair<Eigen::Vector3d, Eigen::Vector3d>{Eigen::Vector3d(vector.x(), 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)},
		std::pair<Eigen::Vector3d, Eigen::Vector3d>{Eigen::Vector3d(0.0, vector.y(), 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)},
		std::pair<Eigen::Vector3d, Eigen::Vector3d>{Eigen::Vector3d(0.0, 0.0, vector.z()), Eigen::Vector3d(0.0, 0.0, 1.0)},
	};

	for(const auto & config : arrow_configs){
		auto marker_msg = getDefaultArrowMsg(marker_id++, msg_time);
		marker_msg.points.push_back(geometry_msgs::msg::Point{});
		geometry_msgs::msg::Point p1{};
		p1.x = scale * config.first.x();
		p1.y = scale * config.first.y();
		p1.z = scale * config.first.z();
		marker_msg.points.push_back(p1);
		marker_msg.color.r = config.second.x();
		marker_msg.color.g = config.second.y();
		marker_msg.color.b = config.second.z();
		marker_msg.color.a = 1.0;
		viz_msg.markers.push_back(marker_msg);
	}
	return viz_msg;
}

} // namespace ghost_sensing

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_sensing::IMUFilterNode>());
	rclcpp::shutdown();
	return 0;
}