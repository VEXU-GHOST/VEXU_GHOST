// Copyright 2017 - 2018 joydeepb@cs.umass.edu, slane@cs.umass.edu,
// kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#ifndef SRC_MATH_GEOMETRY_H_
#define SRC_MATH_GEOMETRY_H_

#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"

using std::sqrt;
using math_util::Sq;
using std::cout;
using std::endl;

namespace geometry {

const float kEpsilon = 1e-6;

// Returns a unit vector in the direction of the given angle.
template <typename T>
Eigen::Matrix<T, 2, 1> Heading(T angle) {
  return Eigen::Matrix<T, 2, 1>(cos(angle), sin(angle));
}

// Returns a vector of the same magnitude, but rotated by 90 degrees counter-
// clockwise.
template <typename T>
Eigen::Matrix<T, 2, 1> Perp(const Eigen::Matrix<T, 2, 1>& v) {
  return Eigen::Matrix<T, 2, 1>(-v.y(), v.x());
}

// Returns the value of the 2D cross product between v1 and v2.
template <typename T>
T Cross(const Eigen::Matrix<T, 2, 1>& v1, const Eigen::Matrix<T, 2, 1>& v2) {
  return (v1.x() * v2.y() - v2.x() * v1.y());
}

template <typename T, int N>
Eigen::Matrix<T, N, 1> GetNormalizedOrZero(const Eigen::Matrix<T, N, 1>& vec) {
  const auto norm = vec.template lpNorm<1>();
  if (norm == 0) {
    return vec;
  } else {
    return vec.normalized();
  }
}

template <typename T, int N>
T GetNormOrZero(const Eigen::Matrix<T, N, 1>& vec) {
  const auto norm = vec.template lpNorm<1>();
  if (norm == T(0)) {
    return 0;
  } else {
    return vec.template lpNorm<2>();
  }
}

// Returns true if v1 is parallel to v2
template <typename T>
bool IsParallel(const Eigen::Matrix<T, 2, 1>& v1,
                const Eigen::Matrix<T, 2, 1>& v2) {
  T l1 = v1.norm();
  T l2 = v2.norm();

  return fabs(fabs(v1.dot(v2)) - l1 * l2) < kEpsilon;
}

// Returns true if the line going from p1 to p2 is parallel to the line from p3
// to p4
template <typename T>
bool IsParallel(const Eigen::Matrix<T, 2, 1>& p1,
                const Eigen::Matrix<T, 2, 1>& p2,
                const Eigen::Matrix<T, 2, 1>& p3,
                const Eigen::Matrix<T, 2, 1>& p4) {
  const Eigen::Matrix<T, 2, 1> v1 = p2 - p1;
  const Eigen::Matrix<T, 2, 1> v2 = p4 - p3;
  const T l1 = v1.norm();
  const T l2 = v2.norm();

  return fabs(fabs(v1.dot(v2)) - l1 * l2) < kEpsilon;
}

// Returns true if v1 is perpendicular to v2
template <typename T>
bool IsPerpendicular(const Eigen::Matrix<T, 2, 1>& v1,
                     const Eigen::Matrix<T, 2, 1>& v2) {
  return fabs(v1.dot(v2)) < kEpsilon;
}

// Returns the two tangent points t0, t1 from the point p, to a circle with
// center c, with radius r.
template <typename T>
void GetTangentPoints(const Eigen::Matrix<T, 2, 1>& p,
                      const Eigen::Matrix<T, 2, 1>& c, const T& r,
                      Eigen::Matrix<T, 2, 1>* right_tangent,
                      Eigen::Matrix<T, 2, 1>* left_tangent) {
  DCHECK_NE(right_tangent, (static_cast<Eigen::Matrix<T, 2, 1>*>(nullptr)));
  DCHECK_NE(left_tangent, (static_cast<Eigen::Matrix<T, 2, 1>*>(nullptr)));

  // Vector from c to p, which will then be normalized.
  Eigen::Matrix<T, 2, 1> c_p_dir = c - p;
  // Squared distance of c to p.
  const T c_p_sqdist = c_p_dir.squaredNorm();
  // Distance of c to p.
  const T c_p_dist = sqrt(c_p_sqdist);
  c_p_dir = c_p_dir / c_p_dist;
  // Distance of tangent point to p.
  const T t_p_dist = sqrt(c_p_sqdist - math_util::Sq(r));
  // Unit vector perpendicular to the line joining c and p.
  const Eigen::Matrix<T, 2, 1> c_p_perp = Perp(c_p_dir);
  // Cosine of the angle between the line from c to the tangent point t0, and
  // the line joining c to p.
  const T cos_t = r / c_p_dist;
  // Sine  of the angle between the line from c to the tangent point t0, and
  // the line joining c to p.
  const T sin_t = t_p_dist / c_p_dist;
  // Projected location of the tangent points onto the line joining c and p.
  const Eigen::Matrix<T, 2, 1> t_projected = c - cos_t * r * c_p_dir;
  *right_tangent = t_projected - sin_t * r * c_p_perp;
  *left_tangent = t_projected + sin_t * r * c_p_perp;
}

// Returns true if point c is between points a and b on a line.
template <typename T>
bool IsBetween(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b,
               const Eigen::Matrix<T, 2, 1>& c, const T epsilon) {
  const T dist = (a - c).norm() + (c - b).norm() - (a - b).norm();
  return dist <= epsilon;
}

// Checks to see if two line segments, (a1-a0) and (b1-b0), cross or touch.
// "Collision" == "crossing" || "touching"
template <typename T>
bool CheckLineLineCollision(
    const Eigen::Matrix<T, 2, 1>& a0, const Eigen::Matrix<T, 2, 1>& a1,
    const Eigen::Matrix<T, 2, 1>& b0, const Eigen::Matrix<T, 2, 1>& b1) {
  const Eigen::Matrix<T, 2, 1> a1a0 = a1 - a0;
  const Eigen::Matrix<T, 2, 1> a0a1 = -a1a0;
  const Eigen::Matrix<T, 2, 1> b0a0 = b0 - a0;
  const Eigen::Matrix<T, 2, 1> b1a0 = b1 - a0;
  const Eigen::Matrix<T, 2, 1> b1a1 = b1 - a1;
  const Eigen::Matrix<T, 2, 1> b1b0 = b1 - b0;
  const Eigen::Matrix<T, 2, 1> b0b1 = -b1b0;
  const Eigen::Matrix<T, 2, 1> a0b0 = a0 - b0;
  const Eigen::Matrix<T, 2, 1> a1b0 = a1 - b0;
  const Eigen::Matrix<T, 2, 1> b0a1 = -a1b0;
  const Eigen::Matrix<T, 2, 1> a0b1 = -b1a0;
  const Eigen::Matrix<T, 2, 1> a1b1 = a1 - b1;
  const int s1 = math_util::Sign(Cross(a1a0, b0a0));
  const int s2 = math_util::Sign(Cross(a1a0, b1a0));
  const int s3 = math_util::Sign(Cross(b1b0, a0b0));
  const int s4 = math_util::Sign(Cross(b1b0, a1b0));
  // Points on opposite sides for both lines.
  if ((s1 != s2) && (s3 != s4)) {
    return true;
  }
  // Line is on the same side of another line.
  if ((s1 + s2 != 0) || (s3 + s4 != 0)) {
    return false;
  }

  const bool a1a0_is_point = (a1a0 == Eigen::Matrix<T, 2, 1>::Zero());
  const bool b1b0_is_point = (b1b0 == Eigen::Matrix<T, 2, 1>::Zero());

  if (a1a0_is_point && b1b0_is_point) {
    if (a1 == b1) {
      return true;
    }
  }

  // Colinear.
  if (!a1a0_is_point) {
    const T a1a0_dot_b0a0 = a1a0.dot(b0a0);
    if (a1a0_dot_b0a0 >= 0 && a1a0_dot_b0a0 <= a1a0.squaredNorm()) {
      return true;
    }
    const T a0a1_dot_b0a1 = a0a1.dot(b0a1);
    if (a0a1_dot_b0a1 >= 0 && a0a1_dot_b0a1 <= a0a1.squaredNorm()) {
      return true;
    }

    const T a1a0_dot_b1a0 = a1a0.dot(b1a0);
    if (a1a0_dot_b1a0 >= 0 && a1a0_dot_b1a0 <= a1a0.squaredNorm()) {
      return true;
    }
    const T a0a1_dot_b1a1 = a0a1.dot(b1a1);
    if (a0a1_dot_b1a1 >= 0 && a0a1_dot_b1a1 <= a0a1.squaredNorm()) {
      return true;
    }
  }

  if (!b1b0_is_point) {
    const T b1b0_dot_a0b0 = b1b0.dot(a0b0);
    if (b1b0_dot_a0b0 >= 0 && b1b0_dot_a0b0 <= b1b0.squaredNorm()) {
      return true;
    }
    const T b0b1_dot_a0b1 = b0b1.dot(a0b1);
    if (b0b1_dot_a0b1 >= 0 && b0b1_dot_a0b1 <= b0b1.squaredNorm()) {
      return true;
    }

    const T b1b0_dot_a1b0 = b1b0.dot(a1b0);
    if (b1b0_dot_a1b0 >= 0 && b1b0_dot_a1b0 <= b1b0.squaredNorm()) {
      return true;
    }
    const T b0b1_dot_a1b1 = b0b1.dot(a1b1);
    if (b0b1_dot_a1b1 >= 0 && b0b1_dot_a1b1 <= b0b1.squaredNorm()) {
      return true;
    }
  }
  return false;
}

// Returns the point of intersection between the lines given by (a1-a0) and
// (b1-b0).
template <typename T>
Eigen::Matrix<T, 2, 1> LineLineIntersection(const Eigen::Matrix<T, 2, 1>& a0,
                                            const Eigen::Matrix<T, 2, 1>& a1,
                                            const Eigen::Matrix<T, 2, 1>& b0,
                                            const Eigen::Matrix<T, 2, 1>& b1) {
  // Vector form of solution taken from:
  // http://mathworld.wolfram.com/Line-LineIntersection.html
  const auto& x1 = a0;
  const auto& x2 = a1;
  const auto& x3 = b0;
  const auto& x4 = b1;
  const Eigen::Matrix<T, 2, 1> a = (x2 - x1);
  const Eigen::Matrix<T, 2, 1> b = (x4 - x3);
  const Eigen::Matrix<T, 2, 1> c = (x3 - x1);
  // Simplifies to remove one call of Cross(a, b) from the numerator and
  // denominator.
  const T numerator = Cross(c, b);
  const T denominator = Cross(a, b);
  return x1 + a * (numerator / denominator);
}

// Checks for intersection between the lines given by (a1-a0) and
// (b1-b0).
//
// If one is found, then it will be returned as the second of the pair.
// If one is not found, then an arbitrary point will be returned.
template <typename T>
std::pair<bool, Eigen::Matrix<T, 2, 1>> CheckLineLineIntersection(
    const Eigen::Matrix<T, 2, 1>& a0, const Eigen::Matrix<T, 2, 1>& a1,
    const Eigen::Matrix<T, 2, 1>& b0, const Eigen::Matrix<T, 2, 1>& b1) {
  if (!CheckLineLineCollision(a0, a1, b0, b1)) {
    return {false, {std::numeric_limits<T>::max(),
                    std::numeric_limits<T>::max()}};
  }
  return {true,
          LineLineIntersection(a0, a1, b0, b1)};
}

// Returns the angle corresponding to the direction of this vector.
template <typename T>
T Angle(const Eigen::Matrix<T, 2, 1>& v) {
  return (atan2(v.y(), v.x()));
}

// Project a given point onto the line passing through vertex_a and vertex_b.
template <typename T>
void ProjectPointOntoLine(const Eigen::Matrix<T, 2, 1>& point,
                          const Eigen::Matrix<T, 2, 1>& vertex_a,
                          const Eigen::Matrix<T, 2, 1>& vertex_b,
                          Eigen::Matrix<T, 2, 1>* projected_point) {
  const Eigen::Matrix<T, 2, 1> line_dir = (vertex_b - vertex_a).normalized();
  // const Eigen::Matrix<T, 2, 1> line_perp = Perp<T>(line_dir);
  *projected_point = vertex_a + line_dir * line_dir.dot(point - vertex_a);
}

// Project a given point onto the line passing through vertex_a and vertex_b.
template <typename T>
Eigen::Matrix<T, 2, 1> ProjectPointOntoLine(
    const Eigen::Matrix<T, 2, 1>& point,
    const Eigen::Matrix<T, 2, 1>& vertex_a,
    const Eigen::Matrix<T, 2, 1>& vertex_b) {
  const Eigen::Matrix<T, 2, 1> line_dir = (vertex_b - vertex_a).normalized();
  return vertex_a + line_dir * line_dir.dot(point - vertex_a);
}

// Project a given point onto a line segment.
// The line segment is defined by its two vertices vertex_a and vertex_b
// The pointers projected_point and distance define the return values
// Projected point is the location ot the projection.
// Squared Distance is the squared distance from the original point to the
// starting point.
template <typename T>
void ProjectPointOntoLineSegment(const Eigen::Matrix<T, 2, 1>& point,
                                 const Eigen::Matrix<T, 2, 1>& vertex_a,
                                 const Eigen::Matrix<T, 2, 1>& vertex_b,
                                 Eigen::Matrix<T, 2, 1>* projected_point,
                                 float* squared_distance) {
  Eigen::Matrix<T, 2, 1> line_segment = vertex_b - vertex_a;
  Eigen::Matrix<T, 2, 1> point_vector = point - vertex_a;

  // Project the point onto the line segment, capping it at the two end points
  float scalar_projection =
      point_vector.dot(line_segment) / line_segment.squaredNorm();

  scalar_projection = std::max<T>(T(0), std::min<T>(T(1), scalar_projection));

  *projected_point = vertex_a + scalar_projection * line_segment;
  *squared_distance = (point - (*projected_point)).squaredNorm();
}

// Closest distance of a point from a line segment.
template <typename T>
T DistanceFromLineSegment(
    const Eigen::Matrix<T, 2, 1>& point,
    const Eigen::Matrix<T, 2, 1>& vertex_a,
    const Eigen::Matrix<T, 2, 1>& vertex_b) {
  Eigen::Matrix<T, 2, 1> dir = (vertex_b - vertex_a);
  const T l = dir.norm();
  dir = dir / l;
  const T c = dir.dot(point - vertex_a);
  if (c < T(0)) {
    return (point - vertex_a).norm();
  } else if (c > l) {
    return (point - vertex_b).norm();
  }
  const Eigen::Matrix<T, 2, 1> n = Perp(dir);
  return fabs(n.dot(point - vertex_a));
}

// Project a given point onto a line segment.
// The line segment is defined by its two vertices vertex_a and vertex_b
// Returns the location of the projection.
template <typename T>
Eigen::Matrix<T, 2, 1> ProjectPointOntoLineSegment(
    const Eigen::Matrix<T, 2, 1>& point,
    const Eigen::Matrix<T, 2, 1>& vertex_a,
    const Eigen::Matrix<T, 2, 1>& vertex_b) {
  Eigen::Matrix<T, 2, 1> line_segment = vertex_b - vertex_a;
  Eigen::Matrix<T, 2, 1> point_vector = point - vertex_a;

  // Project the point onto the line segment, capping it at the two end points
  float scalar_projection =
      point_vector.dot(line_segment) / line_segment.squaredNorm();

  scalar_projection = std::max<T>(T(0), std::min<T>(T(1), scalar_projection));

  return vertex_a + scalar_projection * line_segment;
}

// Determines whether a ray intersects with a given line segment.
// Returns true if they intersect and false otherwise.
// ray_soruce is the origin point of the ray_direction
// ray_direction is a vector that indicates the direction that the ray is going
// in
// line_start is the start point of the line segment
// line_end is the end point of the line segment
// distance is defined to be the distance from the ray source to the intersect
// If there is no intersection, it will be set to -1
// intersect_point is the point at which the ray intersects the line segment
// It will be set to -1, -1 if there is no collision
template <typename T>
bool RayIntersect(const Eigen::Matrix<T, 2, 1>& ray_source,
                  const Eigen::Matrix<T, 2, 1>& ray_direction,
                  const Eigen::Matrix<T, 2, 1>& segment_start,
                  const Eigen::Matrix<T, 2, 1>& segment_end,
                  T* squared_distance,
                  Eigen::Matrix<T, 2, 1>* intersect_point) {
  bool intersects = false;
  *squared_distance = -1;
  intersect_point->x() = -1;
  intersect_point->y() = -1;

  Eigen::Matrix<T, 2, 1> line_to_source = ray_source - segment_start;
  Eigen::Matrix<T, 2, 1> line_direction = segment_end - segment_start;
  Eigen::Matrix<T, 2, 1> perpendicular = Perp(ray_direction);

  T denomenator = line_direction.dot(perpendicular);

  T ray_intersect_param = Cross(line_direction, line_to_source) / denomenator;

  T line_intersect_param = line_to_source.dot(perpendicular) / denomenator;

  if (ray_intersect_param >= T(0) && line_intersect_param >= T(0) &&
      line_intersect_param <= T(1)) {
    intersects = true;
    *intersect_point = (segment_start + line_intersect_param * line_direction);
    *squared_distance = ((*intersect_point) - ray_source).squaredNorm();
  }
  return intersects;
}

template <typename T>
bool RayIntersect(const Eigen::Matrix<T, 2, 1>& ray_source,
                  const Eigen::Matrix<T, 2, 1>& ray_direction,
                  const Eigen::Matrix<T, 2, 1>& line_start,
                  const Eigen::Matrix<T, 2, 1>& line_end) {
  bool intersects = false;

  Eigen::Matrix<T, 2, 1> line_to_source = ray_source - line_start;
  Eigen::Matrix<T, 2, 1> line_direction = line_end - line_start;
  Eigen::Matrix<T, 2, 1> perpendicular = Perp(ray_direction);

  T denomenator = line_direction.dot(perpendicular);

  T ray_intersect_param = Cross(line_direction, line_to_source) / denomenator;

  T line_intersect_param = line_to_source.dot(perpendicular) / denomenator;

  if (ray_intersect_param >= T(0) && line_intersect_param >= T(0) &&
      line_intersect_param <= T(1)) {
    intersects = true;
  }

  return intersects;
}

// Determines whether a ray intersects with a given circle

template <typename T>
bool FurthestFreePointCircle(const Eigen::Matrix<T, 2, 1>& line_start,
                             const Eigen::Matrix<T, 2, 1>& line_end,
                             const Eigen::Matrix<T, 2, 1>& circle_center,
                             const T radius, T* squared_distance,
                             Eigen::Matrix<T, 2, 1>* free_point) {
  bool collides = false;

  if ((line_start - circle_center).squaredNorm() < Sq(radius)) {
    *free_point = line_start;
    collides = true;
  } else {
    Eigen::Matrix<T, 2, 1> projected_point;
    T current_distance;
    ProjectPointOntoLineSegment(circle_center, line_start, line_end,
                                &projected_point, &current_distance);

    if ((projected_point - circle_center).squaredNorm() >= Sq(radius)) {
      *free_point = line_end;
      collides = false;
    } else {
      collides = true;
      Eigen::Matrix<T, 2, 1> translation_vector = projected_point - line_start;
      T slide_back_distance =
          sqrt(Sq(radius) - (projected_point - circle_center).squaredNorm());
      translation_vector -=
          (translation_vector.normalized()) * slide_back_distance;

      *free_point = line_start + translation_vector;
    }
  }

  *squared_distance = ((*free_point) - line_start).squaredNorm();
  return collides;
}

template <typename T>
T MinDistanceLineLine(const Eigen::Matrix<T, 2, 1>& a0,
                      const Eigen::Matrix<T, 2, 1>& a1,
                      const Eigen::Matrix<T, 2, 1>& b0,
                      const Eigen::Matrix<T, 2, 1>& b1) {
  if (CheckLineLineCollision(a0, a1, b0, b1)) {
    return 0;
  }

  const auto a0_proj = ProjectPointOntoLineSegment(a0, b0, b1);
  const auto a1_proj = ProjectPointOntoLineSegment(a1, b0, b1);
  const auto b0_proj = ProjectPointOntoLineSegment(b0, a0, a1);
  const auto b1_proj = ProjectPointOntoLineSegment(b1, a0, a1);

  const auto a0_diff_sq = (a0 - a0_proj).squaredNorm();
  const auto a1_diff_sq = (a1 - a1_proj).squaredNorm();
  const auto b0_diff_sq = (b0 - b0_proj).squaredNorm();
  const auto b1_diff_sq = (b1 - b1_proj).squaredNorm();

  const auto min_diff_sq =
      std::min({a0_diff_sq, a1_diff_sq, b0_diff_sq, b1_diff_sq});
  return std::sqrt(min_diff_sq);
}

// Compute point(s) of intersection between the circle with center c0 and radius
// r, and the line segment p0 : p1. The return value indicates the number of
// valid points of intersection found: 0, 1, or 2. The points of intersection
// are returned via r0 and r1.
template <typename T>
int CircleLineIntersection(const Eigen::Matrix<T, 2, 1>& c0,
                           const T& r,
                           const Eigen::Matrix<T, 2, 1>& p0,
                           const Eigen::Matrix<T, 2, 1>& p1,
                           Eigen::Matrix<T, 2, 1>* r0,
                           Eigen::Matrix<T, 2, 1>* r1) {
  Eigen::Matrix<T, 2, 1> d = p1 - p0;
  const T l = d.norm();
  d = d / l;
  const Eigen::Matrix<T, 2, 1> delta = p0 - c0;  
  const T b = T(2) * delta.dot(d);
  const T c = delta.squaredNorm() - math_util::Sq(r);
  T gamma_0;
  T gamma_1;
  const int num_solutions = 
      math_util::SolveQuadratic(T(1), b, c, &gamma_0, &gamma_1);
  if (num_solutions == 0) return 0;
  bool s0_valid =  (gamma_0 >= T(0) && gamma_0 <= l);
  bool s1_valid = (num_solutions > 1 && gamma_1 >= T(0) && gamma_1 <= l);
  if (!s0_valid && s1_valid) {
    s0_valid = true;
    gamma_0 = gamma_1;
    s1_valid = false;
  }
  if (s0_valid) {
    *r0 = p0 + gamma_0 * d;
  }
  if (s1_valid) {
    *r1 = p0 + gamma_1 * d;
  }
  return static_cast<int>(s0_valid) + static_cast<int>(s1_valid);
}
// Check if the circle with center c0, and radius r collides with the line
// segment p0 : p1, and return true if a collision is found.
template <typename T>
bool CheckCircleLineCollision(const Eigen::Matrix<T, 2, 1>& c0,
                              const T& r,
                              const Eigen::Matrix<T, 2, 1>& p0,
                              const Eigen::Matrix<T, 2, 1>& p1) {
  if ((c0 - p0).squaredNorm() <= math_util::Sq(r)) return true;
  if ((c0 - p1).squaredNorm() <= math_util::Sq(r)) return true;
  if (DistanceFromLineSegment(c0, p0, p1) <= r) return true;
  return false;
}

// Check if line segment collides min and max angle on a circle, moving in a
// counterclockwise direction from min_angle to max_angle.
template <typename T>
T MinDistanceLineArc(const Eigen::Matrix<T, 2, 1>& l0,
                     const Eigen::Matrix<T, 2, 1>& l1,
                     const Eigen::Matrix<T, 2, 1>& a_center,
                     const T& a_radius,
                     T a_angle_start,
                     T a_angle_end,
                     const int rotation_sign) {
  a_angle_start = math_util::AngleMod(a_angle_start);
  a_angle_end = math_util::AngleMod(a_angle_end);
  const auto proj_center_segment =
      ProjectPointOntoLineSegment(a_center, l0, l1);
  const auto proj_dir_segment = proj_center_segment - a_center;

  const auto l0_dir = l0 - a_center;
  const auto l1_dir = l1 - a_center;

  if (proj_dir_segment.squaredNorm() >= Sq(a_radius) ||
      (l0_dir.squaredNorm() < Sq(a_radius) &&
       l1_dir.squaredNorm() < Sq(a_radius))) {
    // Out of the circle, tangent to circle, or completely in circle.
    const T angle = math_util::AngleMod(Angle<T>(proj_dir_segment));
    if (math_util::IsAngleBetween(
        angle, a_angle_start, a_angle_end, rotation_sign)) {
      // Vector of the center projected onto line segment is in arc.
      const float distance_to_proj = (proj_dir_segment.norm() - a_radius);
      if (distance_to_proj >= T(0)) {
        return distance_to_proj;
      }
    }
    // Vector of the center projected onto line segment is not in arc.
    const auto a_angle_min_v = Heading(a_angle_start) * a_radius;
    const auto a_angle_max_v = Heading(a_angle_end) * a_radius;
    return std::sqrt(
        std::min((proj_dir_segment - a_angle_min_v).squaredNorm(),
                 (proj_dir_segment - a_angle_max_v).squaredNorm()));
  }

  const auto proj_center_line = ProjectPointOntoLine(a_center, l0, l1);
  const auto proj_dir_line = proj_center_line - a_center;
  const T along_line_dist =
      std::sqrt(Sq(a_radius) - proj_dir_line.squaredNorm());

  static const auto direction_line_intersects_arc =
      [](const Eigen::Matrix<T, 2, 1>& proj_center_line,
         const T& along_line_dist,
         const T& a_angle_start,
         const T& a_angle_end,
         const Eigen::Matrix<T, 2, 1>& a_center,
         const int& rotation_sign,
         const Eigen::Matrix<T, 2, 1>& direction_line) -> bool {
        const auto circle_intersect_point =
            direction_line * along_line_dist + proj_center_line;

        const T angle = math_util::AngleMod(
            Angle<T>(circle_intersect_point - a_center));
        return math_util::IsAngleBetween(
            angle, a_angle_start, a_angle_end, rotation_sign);
      };

  static const auto no_arc_intersect_min_distance =
      [](const Eigen::Matrix<T, 2, 1>& a_center,
         const T& a_angle_start,
         const T& a_angle_end,
         const T& a_radius,
         const Eigen::Matrix<T, 2, 1>& l0,
         const Eigen::Matrix<T, 2, 1>& l1) -> float {
    const auto a_angle_min_v = Heading(a_angle_start) * a_radius + a_center;
    const auto a_angle_max_v = Heading(a_angle_end) * a_radius + a_center;
    const auto proj_a_angle_min_v =
        ProjectPointOntoLineSegment<T>(a_angle_min_v, l0, l1);
    const auto proj_a_angle_max_v =
        ProjectPointOntoLineSegment<T>(a_angle_max_v, l0, l1);
    const Eigen::Matrix<T, 2, 1> del_min_v = proj_a_angle_min_v - a_angle_min_v;
    const T del_min_v_sq_norm = del_min_v.squaredNorm();
    const Eigen::Matrix<T, 2, 1> del_max_v = proj_a_angle_max_v - a_angle_max_v;
    const T del_max_v_sq_norm = del_max_v.squaredNorm();
    const T min_dist =
        std::sqrt(std::min(del_min_v_sq_norm, del_max_v_sq_norm));
    return min_dist;
  };

  // l0 inside the circle, l1 outside the circle.
  if (l0_dir.squaredNorm() < Sq(a_radius) &&
      l1_dir.squaredNorm() >= Sq(a_radius)) {
    const auto direction_line =
        GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l1 - l0));
    if (direction_line_intersects_arc(proj_center_line,
                                      along_line_dist,
                                      a_angle_start,
                                      a_angle_end,
                                      a_center,
                                      rotation_sign,
                                      direction_line)) {
      // Arc intersects with line.
      return 0;
    }

    return no_arc_intersect_min_distance(
        a_center, a_angle_start, a_angle_end, a_radius, l0, l1);
  }

  // l1 inside the circle, l0 outside the circle.
  if (l0_dir.squaredNorm() >= Sq(a_radius) &&
      l1_dir.squaredNorm() < Sq(a_radius)) {
    const auto direction_line =
        GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l0 - l1));
    if (direction_line_intersects_arc(proj_center_line,
                                      along_line_dist,
                                      a_angle_start,
                                      a_angle_end,
                                      a_center,
                                      rotation_sign,
                                      direction_line)) {
      // Arc intersects with line.
      return 0;
    }
    return no_arc_intersect_min_distance(
        a_center, a_angle_start, a_angle_end, a_radius, l0, l1);
  }

  // Both l0 and l1 are not in the circle, but the line goes through the circle.
  const auto direction_line =
      GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l1 - l0));

  const bool positive_direction_intersect =
      direction_line_intersects_arc(proj_center_line,
                                    along_line_dist,
                                    a_angle_start,
                                    a_angle_end,
                                    a_center,
                                    rotation_sign,
                                    direction_line);
  const bool negative_direction_intersect =
      direction_line_intersects_arc(proj_center_line,
                                    along_line_dist,
                                    a_angle_start,
                                    a_angle_end,
                                    a_center,
                                    rotation_sign,
                                    -direction_line);

  if (positive_direction_intersect || negative_direction_intersect) {
    // Arc intersects with line.
    return 0;
  }

  const T min_distance = no_arc_intersect_min_distance(
      a_center, a_angle_start, a_angle_end, a_radius, l0, l1);
  return min_distance;
}

// Returns the scalar projection of vector1 onto vector2
// vector2 must be nonzero
template <typename T>
T ScalarProjection(const Eigen::Matrix<T, 2, 1>& vector1,
                   const Eigen::Matrix<T, 2, 1>& vector2) {
  return vector1.dot(vector2)/vector2.norm();
}

}  // namespace geometry

#endif  // SRC_MATH_GEOMETRY_H_
