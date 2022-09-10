/*
 * Filename: test_gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday July 18th 2022 11:41:53 am
 * Modified By: Maxx Wilson
 */

#include "simulator_plugins/gazebo_swerve_plugin.hpp"

#include <gazebo/test/ServerFixture.hh>
#include <gazebo/common/Time.hh>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

class GazeboSwervePluginTest : public gazebo::ServerFixture
{
public:

  void SwerveBringup()
  {
    // ROS Node and Callback Executor
    ros_node_ = std::make_shared<rclcpp::Node>("gazebo_swerve_plugin_test_node");

    // Create publisher
    wheel_input_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
        "wheel_voltage",
        rclcpp::SensorDataQoS());

    steering_input_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>(
        "steering_voltage",
        rclcpp::SensorDataQoS());

    // Create subscriber 
    wheel_torque_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        "wheel_torque", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {
          wheel_torque_msg_ = msg;
        });

    steering_torque_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        "steering_torque", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {
          steering_torque_msg_ = msg;
        });

    auto pkg_share = ament_index_cpp::get_package_share_directory("swerve_drive");
    auto world_file_path = pkg_share + "/urdf/spin_up.world";
    auto sdf_file_path = pkg_share + "/urdf/swerve.sdf";

    this->Load(world_file_path, true);

    std::ifstream sdf_file(sdf_file_path);
    std::stringstream buffer;
    buffer << sdf_file.rdbuf();

    this->SpawnSDF(buffer.str());
    this->WaitUntilEntitySpawn("swerve", 100, 100);

    // World
    world_ = gazebo::physics::get_world();

    // Model
    model_ = world_->ModelByName("swerve");

    // Initialize Gazebo with a few starting steps
    world_->Step(500);
    gazebo::common::Time::MSleep(100);
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;

  rclcpp::Node::SharedPtr ros_node_;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3>> wheel_input_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3>> steering_input_pub_;
  
  // Subscribers
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> wheel_torque_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> steering_torque_sub_;

  // Msgs
  geometry_msgs::msg::Vector3::SharedPtr input_wheel_msg_;
  geometry_msgs::msg::Vector3::SharedPtr input_steering_msg_;
  geometry_msgs::msg::Vector3::SharedPtr wheel_torque_msg_;
  geometry_msgs::msg::Vector3::SharedPtr steering_torque_msg_;
};

TEST_F(GazeboSwervePluginTest, testJointTorque)
{
  SwerveBringup();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ros_node_);
  executor.spin_once(100ms);

  auto input_test_msg = geometry_msgs::msg::Vector3{};
  input_test_msg.x = 1.0;
  input_test_msg.y = 1.0;
  input_test_msg.z = 1.0;
  wheel_input_pub_->publish(input_test_msg);

  for (unsigned int i = 0; i < 100; ++i) {
    world_->Step(100);
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }

  EXPECT_TRUE(model_->GetJoint("driveshaft_1")->GetVelocity(2) > 0.0);
  EXPECT_TRUE(model_->GetJoint("driveshaft_2")->GetVelocity(2) > 0.0);
  EXPECT_TRUE(model_->GetJoint("driveshaft_3")->GetVelocity(2) > 0.0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}