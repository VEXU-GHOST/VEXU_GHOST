// ========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
// ========================================================================
/*!
   \file    particle-filter-main.cc
   \brief   Main entry point for particle filter based
         mobile robot localization
   \author  Joydeep Biswas, (C) 2019
 */
// ========================================================================

#include "eigen3/Eigen/Geometry"
#include "ghost_localization/ekf_pf_node.hpp"
#include "ghost_util/angle_util.hpp"
#include "math/line2d.h"
#include "math/math_util.h"
#include "util/timer.h"

using Eigen::Vector2f;
using geometry::Line;
using geometry::Line2f;
using math_util::DegToRad;
using math_util::RadToDeg;
using particle_filter::ParticleFilter;
using particle_filter::ParticleFilterConfig;
using std::placeholders::_1;
using std::string;
using std::vector;

namespace ghost_localization {

EkfPfNode::EkfPfNode() :
	Node("ekf_pf_node"){
	// Subscribers
	ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/odometry/filtered",
		10,
		std::bind(&EkfPfNode::EkfCallback, this, _1));
	// Publishers
}

void EkfPfNode::EkfCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
}

} // namespace ghost_localization

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_localization::EkfPfNode>());
	rclcpp::shutdown();
	return 0;
}