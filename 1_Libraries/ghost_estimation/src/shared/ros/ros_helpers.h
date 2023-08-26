// Copyright 2019 joydeepb@cs.utexas.edu
// Computer Science Department
// University of Texas at Austin
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

// C++ headers.
#include <string>

// C++ Library headers.
#include "eigen3/Eigen/Core"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"

// Custom headers.
#include "math/math_util.h"

#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

namespace ros_helpers {

inline void InitRosHeader(const std::string& frame_id,
                          std_msgs::Header* h) {
  h->seq = 0;
  h->frame_id = frame_id;
  h->stamp = ros::Time::now();
}

inline void ClearMarker(visualization_msgs::Marker* m) {
  m->points.clear();
  m->colors.clear();
}

template<typename Tr, typename Tg, typename Tb, typename Ta>
std_msgs::ColorRGBA RosColor(const Tr& r,
                             const Tg& g,
                             const Tb& b,
                             const Ta& a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

template<typename Tr, typename Tg, typename Tb, typename Ta, typename RosColor>
void SetRosColor(const Tr& r,
                 const Tg& g,
                 const Tb& b,
                 const Ta& a,
                 RosColor* c) {
  c->r = r;
  c->g = g;
  c->b = b;
  c->a = a;
}

template<typename Tx, typename Ty, typename Tz>
geometry_msgs::Point RosPoint(const Tx& x, const Ty& y, const Tz& z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

template<typename Tx, typename Ty>
geometry_msgs::Point RosPoint(const Tx& x, const Ty& y) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;
  return p;
}

template<typename Tx, typename Ty, typename Tz, typename RosVector>
void SetRosVector(const Tx& x, const Ty& y, const Tz& z, RosVector* v) {
  v->x = x;
  v->y = y;
  v->z = z;
}

template<typename RosVector, typename T>
void SetRosQuaternion(const T& w,
                      const T& x,
                      const T& y,
                      const T& z,
                      RosVector* q) {
  q->w = w;
  q->x = x;
  q->y = y;
  q->z = z;
}

template<typename RosVector>
void SetIdentityRosQuaternion(RosVector* q) {
  q->w = 1;
  q->x = 0;
  q->y = 0;
  q->z = 0;
}

template <typename Derived>
geometry_msgs::Point Eigen3DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  geometry_msgs::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  return p;
}

template <typename Derived>
geometry_msgs::Point Eigen2DToRosPoint(const Eigen::DenseBase<Derived>& v) {
  geometry_msgs::Point p;
  p.x = v[0];
  p.y = v[1];
  p.z = 0;
  return p;
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     visualization_msgs::Marker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     const std_msgs::ColorRGBA& c,
                     visualization_msgs::Marker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
  msg->colors.push_back(c);
  msg->colors.push_back(c);
}

template <typename Derived>
void DrawEigen2DLine(const Eigen::DenseBase<Derived>& v1,
                     const Eigen::DenseBase<Derived>& v2,
                     const std_msgs::ColorRGBA& c1,
                     const std_msgs::ColorRGBA& c2,
                     visualization_msgs::Marker* msg) {
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v1));
  msg->points.push_back(Eigen2DToRosPoint<Derived>(v2));
  msg->colors.push_back(c1);
  msg->colors.push_back(c2);
}

template <typename Vector2>
void DrawCross(const Vector2& v,
              const float size,
              const std_msgs::ColorRGBA& color,
              visualization_msgs::Marker* msg) {
  msg->points.push_back(Eigen2DToRosPoint(v - Vector2(size, size)));
  msg->points.push_back(Eigen2DToRosPoint(v + Vector2(size, size)));
  msg->points.push_back(Eigen2DToRosPoint(v - Vector2(size, -size)));
  msg->points.push_back(Eigen2DToRosPoint(v + Vector2(size, -size)));
  msg->colors.push_back(color);
  msg->colors.push_back(color);
  msg->colors.push_back(color);
  msg->colors.push_back(color);
}

}  // namespace ros_helpers

#endif  // ROS_HELPERS_H
