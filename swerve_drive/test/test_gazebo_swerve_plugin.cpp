/*
 * Filename: test_gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday July 18th 2022 11:41:53 am
 * Modified By: Maxx Wilson
 */

#include "gazebo_swerve_plugin.hpp"

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
    auto pkg_share = ament_index_cpp::get_package_share_directory("swerve_drive");
    auto world_file_path = pkg_share + "/urdf/spin_up.world";
    auto sdf_file_path = pkg_share + "/urdf/swerve.sdf";

    this->Load(world_file_path, true);

    std::ifstream sdf_file(sdf_file_path);
    std::stringstream buffer;
    buffer << sdf_file.rdbuf();

    this->SpawnSDF(buffer.str());
    this->WaitUntilEntitySpawn("swerve", 100, 100);
  }
};

TEST_F(GazeboSwervePluginTest, testJointTorque)
{
  // Spawn world and model
  this->SwerveBringup();

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto swerve = world->ModelByName("swerve");
  ASSERT_NE(nullptr, swerve);

  // New ROS Node
  auto node = std::make_shared<rclcpp::Node>("gazebo_swerve_plugin_test_node");

  // Make an executor for node callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Initialize Gazebo with a few starting steps
  world->Step(500);
  executor.spin_once(500ms);
  gazebo::common::Time::MSleep(100);

  // Create publisher
  std_msgs::msg::Float32MultiArray::SharedPtr actuator_input_msg;
  auto pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "actuator_inputs",
      rclcpp::SensorDataQoS());

  // Create subscriber
  std_msgs::msg::Float32MultiArray::SharedPtr actuator_output_msg;
  auto sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "actuator_outputs", rclcpp::SensorDataQoS(),
      [&actuator_output_msg](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
      {
        actuator_output_msg = msg;
      });

  auto input_test_msg = std_msgs::msg::Float32MultiArray{};
  input_test_msg.data = std::vector<float>(6);
  input_test_msg.data[0] = 1.0;
  pub->publish(input_test_msg);

  for (unsigned int i = 0; i < 10; ++i) {
    world->Step(100);
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
    
    EXPECT_TRUE(actuator_output_msg->data[0] > 0.0);
    EXPECT_TRUE(swerve->GetJoint("pivot_1")->GetForceTorque(0).body1Torque[2] > 0.0);
    EXPECT_TRUE(swerve->GetJoint("pivot_1")->GetVelocity(2) > 0.0);

    actuator_output_msg.reset();
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}