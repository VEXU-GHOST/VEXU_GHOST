/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <gtest/gtest.h>
#include "ghost_ros_interfaces/msg_helpers/msg_helpers.hpp"
#include "ghost_v5_interfaces/test/device_test_utils.hpp"
#include "ghost_util/test_util.hpp"

using namespace ghost_ros_interfaces::msg_helpers;
using namespace ghost_ros_interfaces;
using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces::test_util;
using namespace ghost_v5_interfaces;
using namespace ghost_util;


TEST(TestMsgHelpers, testMotorStateMsg) {
  auto motor_input = getRandomMotorData(false);
  auto msg = std::make_shared<ghost_msgs::msg::V5MotorState>();
  auto motor_output = std::make_shared<MotorDeviceData>("");

  // Convert to ROS Msg
  toROSMsg(*motor_input, *msg);
  fromROSMsg(*motor_output, *msg);

  EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestMsgHelpers, testMotorCommandMsg) {
  auto motor_input = getRandomMotorData(true);
  auto msg = std::make_shared<ghost_msgs::msg::V5MotorCommand>();
  auto motor_output = std::make_shared<MotorDeviceData>("");

  // Convert to ROS Msg
  toROSMsg(*motor_input, *msg);
  fromROSMsg(*motor_output, *msg);

  EXPECT_EQ(*motor_input, *motor_output);
}

TEST(TestMsgHelpers, testRotationSensorStateMsg) {
  auto rotation_input = getRandomRotationSensorData();
  auto msg = std::make_shared<ghost_msgs::msg::V5RotationSensorState>();
  auto rotation_output = std::make_shared<RotationSensorDeviceData>("");

  toROSMsg(*rotation_input, *msg);
  fromROSMsg(*rotation_output, *msg);

  EXPECT_EQ(*rotation_input, *rotation_output);
}

TEST(TestMsgHelpers, testInertialSensorStateMsg) {
  auto inertial_input = getRandomInertialSensorData();
  auto msg = std::make_shared<ghost_msgs::msg::V5InertialSensorState>();
  auto inertial_output = std::make_shared<InertialSensorDeviceData>("");

  toROSMsg(*inertial_input, *msg);
  fromROSMsg(*inertial_output, *msg);

  EXPECT_EQ(*inertial_input, *inertial_output);
}

TEST(TestMsgHelpers, testJoystickStateMsg) {
  auto joy_input = getRandomJoystickData();
  auto msg = std::make_shared<ghost_msgs::msg::V5JoystickState>();
  auto joy_output = std::make_shared<JoystickDeviceData>("");

  // Convert to ROS Msg
  toROSMsg(*joy_input, *msg);
  fromROSMsg(*joy_output, *msg);

  EXPECT_EQ(*joy_input, *joy_output);
}

TEST(TestDeviceInterfaces, testRobotTrajectoryMsg) {
  auto rt_input = std::make_shared<ghost_planners::RobotTrajectory>();
  auto mt_input = std::make_shared<ghost_planners::RobotTrajectory::Trajectory>();
  mt_input->position_vector.push_back(0);
  rt_input->x_trajectory = *mt_input;


  auto msg = std::make_shared<ghost_msgs::msg::RobotTrajectory>();
  auto rt_output = std::make_shared<ghost_planners::RobotTrajectory>();

  // Convert to ROS Msg
  toROSMsg(*rt_input, *msg);
  fromROSMsg(*rt_output, *msg);

  EXPECT_EQ(*rt_input, *rt_output);
}

TEST(TestMsgHelpers, testLabeledVectorMapUnorderedMapConversion) {
  std::unordered_map<std::string, std::vector<double>> expected_map{
    {"1", std::vector<double>{0.0}},
    {"2", std::vector<double>{1.0, 2.5}},
    {"3", std::vector<double>{1.18, 6.8}}
  };

  ghost_msgs::msg::LabeledVectorMap expected_msg;

  ghost_msgs::msg::LabeledVector e1;
  e1.label = "1";
  e1.data_array = std::vector<double>{0.0};
  expected_msg.entries.push_back(e1);

  ghost_msgs::msg::LabeledVector e2;
  e2.label = "2";
  e2.data_array = std::vector<double>{1.0, 2.5};
  expected_msg.entries.push_back(e2);

  ghost_msgs::msg::LabeledVector e3;
  e3.label = "3";
  e3.data_array = std::vector<double>{1.18, 6.8};
  expected_msg.entries.push_back(e3);

  std::unordered_map<std::string, std::vector<double>> blank_map;
  ghost_msgs::msg::LabeledVectorMap blank_msg;

  toROSMsg(expected_map, blank_msg);
  EXPECT_EQ(expected_msg.entries.size(), blank_msg.entries.size());

  // Maps are unordered, so need custom equality operator
  for (const auto & entry_1 : expected_msg.entries) {
    int found_count = 0;
    // Ensure each entry in first msg has a corresponding unique entry in second msg
    for (const auto & entry_2 : blank_msg.entries) {
      if (entry_1.label == entry_2.label) {
        found_count += 1;
        EXPECT_EQ(entry_1.data_array, entry_2.data_array);
      }
    }
    EXPECT_EQ(found_count, 1);
  }

  fromROSMsg(blank_map, expected_msg);
  EXPECT_EQ(expected_map, blank_map);
}

TEST(TestMsgHelpers, testLabeledVectorMapTrajectoryConversion) {

  ghost_planners::Trajectory trajectory(std::vector<std::string>{"s1", "s2"});
  trajectory.addNode(0.0, std::vector<double>{1.0, 2.0});
  trajectory.addNode(1.0, std::vector<double>{2.0, 1.0});
  trajectory.addNode(2.0, std::vector<double>{2.5, 1.5});

  ghost_msgs::msg::LabeledVectorMap expected_msg;

  ghost_msgs::msg::LabeledVector e1;
  e1.label = "time";
  e1.data_array = std::vector<double>{0.0, 1.0, 2.0};
  expected_msg.entries.push_back(e1);

  ghost_msgs::msg::LabeledVector e2;
  e2.label = "s1";
  e2.data_array = std::vector<double>{1.0, 2.0, 2.5};
  expected_msg.entries.push_back(e2);

  ghost_msgs::msg::LabeledVector e3;
  e3.label = "s2";
  e3.data_array = std::vector<double>{2.0, 1.0, 1.5};
  expected_msg.entries.push_back(e3);

  ghost_planners::Trajectory blank_trajectory(std::vector<std::string>{});
  EXPECT_NO_THROW(fromROSMsg(blank_trajectory, expected_msg));
  EXPECT_EQ(trajectory, blank_trajectory);

  ghost_msgs::msg::LabeledVectorMap blank_msg;
  toROSMsg(trajectory, blank_msg);

  // Check msgs contain identical data
  EXPECT_EQ(expected_msg.entries.size(), blank_msg.entries.size());
  for (const auto & blank_entry : blank_msg.entries) {
    bool found = false;
    for (const auto & expected_entry : expected_msg.entries) {
      if (expected_entry.label == blank_entry.label) {
        found = true;
        EXPECT_EQ(expected_entry.data_array, blank_entry.data_array);
      }
    }
    EXPECT_TRUE(found);
  }
}

TEST(TestMsgHelpers, testIPOPTOutputConversion) {
  ghost_planners::IterationCallback::IPOPTOutput expected_solver_output{};
  ghost_msgs::msg::IPOPTOutput expected_msg{};

  auto cost = getRandomDouble();
  expected_solver_output.cost = cost;
  expected_msg.cost = cost;

  auto iteration = getRandomInt(3000);
  expected_solver_output.iteration = iteration;
  expected_msg.iteration = iteration;

  for (int i = 0; i < 5; i++) {
    auto x = getRandomDouble();
    expected_solver_output.state_vector.push_back(x);
    expected_msg.state_vector.push_back(x);

    auto g = getRandomDouble();
    expected_solver_output.constraint_vector.push_back(g);
    expected_msg.constraint_vector.push_back(g);

    auto lambda_x = getRandomDouble();
    expected_solver_output.state_lagrange_multipliers.push_back(lambda_x);
    expected_msg.state_lagrange_multipliers.push_back(lambda_x);

    auto lambda_g = getRandomDouble();
    expected_solver_output.constraint_lagrange_multipliers.push_back(lambda_g);
    expected_msg.constraint_lagrange_multipliers.push_back(lambda_g);
  }

  ghost_planners::IterationCallback::IPOPTOutput blank_solver_output{};
  ghost_msgs::msg::IPOPTOutput blank_msg{};

  toROSMsg(expected_solver_output, blank_msg);
  EXPECT_EQ(expected_msg, blank_msg);

  fromROSMsg(blank_solver_output, expected_msg);
  EXPECT_EQ(expected_solver_output, blank_solver_output);
}
