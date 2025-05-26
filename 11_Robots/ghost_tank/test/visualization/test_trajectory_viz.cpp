/*
 * Copyright (c) 2025 Maxx Wilson
 * All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ghost_tank/control/trajectory.hpp>
#include <ghost_tank/control/path_generators.hpp>
#include <ghost_tank/visualization/visualization_helpers.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <vector>

class TrajectoryPublisherNode : public rclcpp::Node
{
public:
  TrajectoryPublisherNode()
  : Node("trajectory_publisher_node")
  {
    // Create a publisher for MarkerArray messages on the "trajectory_viz" topic
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_viz", 10);

    // Create a timer to publish the trajectory periodically
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), // Publish every 500 milliseconds
      std::bind(&TrajectoryPublisherNode::publishTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "TrajectoryPublisherNode initialized and ready to publish.");
  }

private:
  void publishTrajectory()
  {
    // Create a MarkerArray message and clear it for this publication cycle
    visualization_msgs::msg::MarkerArray viz_msg;
    viz_msg.markers.clear();

    // Define common parameters for trajectory generation
    int num_points_per_segment = 50;    // Number of points for each bezier segment
    double bezier_lead_distance = 1.0;  // Lead distance for control points
    int num_arrows_to_display = 5;      // Number of arrows for each segment

    // Segment 1: Cubic Bezier from (0, 0, 0 deg) to (2, 2, 0 deg)
    Eigen::Vector2d cubic_start_pos(0.0, 0.0);
    double cubic_start_angle_rad = 0.0 * ghost_util::DEG_TO_RAD; // 0 degrees

    Eigen::Vector2d cubic_end_pos(2.0, 2.0);
    double cubic_end_angle_rad = 0.0 * ghost_util::DEG_TO_RAD;   // 0 degrees

    ghost_tank::motion_planning::Trajectory cubic_segment =
      ghost_tank::motion_planning::generateCubicBezierCurve(
        cubic_start_pos,
        cubic_start_angle_rad,
        cubic_end_pos,
        cubic_end_angle_rad,
        bezier_lead_distance,
        num_points_per_segment
      );

    // Populate MarkerArray with the cubic segment's visualization
    ghost_tank::visualization::getTrajectoryMsg(cubic_segment, viz_msg, num_arrows_to_display);

    // Get the end pose (position and orientation) of the cubic segment to start the quadratic
    Eigen::Vector2d quadratic_start_pos(0.0, 0.0); // Default in case cubic_segment is empty
    if (!cubic_segment.x.empty()) {
        quadratic_start_pos = Eigen::Vector2d(cubic_segment.x.back(), cubic_segment.y.back());
    } else {
        RCLCPP_WARN(this->get_logger(), "Cubic segment is empty, quadratic segment will start at (0,0).");
    }

    // Segment 2: Quadratic Bezier from previous end point to (-1.0, 0.0, 180 deg)
    Eigen::Vector2d quadratic_end_pos(-1.0, 0.0);
    double quadratic_end_angle_rad = 180.0 * ghost_util::DEG_TO_RAD; // 180 degrees

    ghost_tank::motion_planning::Trajectory quadratic_segment =
      ghost_tank::motion_planning::generateQuadraticBezierCurve(
        quadratic_start_pos,
        quadratic_end_pos,
        quadratic_end_angle_rad,
        bezier_lead_distance,
        num_points_per_segment
      );

    // Populate MarkerArray with the quadratic segment's visualization (appends to viz_msg)
    ghost_tank::visualization::getTrajectoryMsg(quadratic_segment, viz_msg, num_arrows_to_display);

    // Publish the MarkerArray message
    marker_array_publisher_->publish(viz_msg);

    RCLCPP_INFO_ONCE(this->get_logger(), "Cubic and quadratic bezier trajectories published to /trajectory_viz topic.");
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TrajectoryPublisherNode>());

  rclcpp::shutdown();
  return 0;
}
