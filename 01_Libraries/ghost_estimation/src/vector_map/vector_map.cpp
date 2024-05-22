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
   \file    vector_map.cpp
   \brief   Vector map representation.
   \author  Joydeep Biswas, (C) 2019
 */
// ========================================================================

#include "stdio.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"

#include "ghost_estimation/vector_map/vector_map.hpp"
#include "math/geometry.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include "util/timer.h"

using Eigen::Vector2f;
using geometry::Cross;
using geometry::Line;
using Line2f = geometry::Line2f;
using math_util::AngleMod;
using math_util::RadToDeg;
using std::string;
using std::swap;
using std::vector;

#define PRINT_LINE(LINE) \
  (LINE).p0.x(), (LINE).p0.y(), \
  (LINE).p1.x(), (LINE).p1.y()

#define PRINT_VEC2(V) (V).x(), (V).y()

DEFINE_double(
  min_line_length,
  0.05,
  "Minimum line length to consider for Analytic ray casting");

namespace vector_map
{

void TrimOcclusion(
  const Vector2f & loc,
  const Line2f & test_line,
  Line2f * trim_line_ptr,
  vector<Line2f> * scene_lines_ptr)
{
  // return TrimOcclusionInt(loc, test_line, trim_line_ptr, scene_lines_ptr);
  static const bool kDebug = false;
  Line2f & trim_line = *trim_line_ptr;
  if (kDebug) {
    printf("%s:\n", __FUNCTION__);
    printf("TestLine: %f,%f %f,%f\n", PRINT_LINE(test_line));
    printf("TrimLine: %f,%f %f,%f\n", PRINT_LINE(trim_line));
    printf("Loc: %f, %f\n", PRINT_VEC2(loc));
  }
  vector<Line2f> & scene_lines = *scene_lines_ptr;
  static const float sqeps = 1e-8;

  // test_line.p0
  Vector2f l1_p0 = test_line.p0;
  // test_line.p1
  Vector2f l1_p1 = test_line.p1;
  // test_line.p0 - loc
  Vector2f l1_r0 = l1_p0 - loc;
  // test_line.p1 - loc
  Vector2f l1_r1 = l1_p1 - loc;
  // trim_line.p0
  Vector2f l2_p0 = trim_line.p0;
  // trim_line.p1
  Vector2f l2_p1 = trim_line.p1;
  // trim_line.p0 - loc
  Vector2f l2_r0 = l2_p0 - loc;
  // trim_line.p1 - loc
  Vector2f l2_r1 = l2_p1 - loc;

  // Ensure that r0 vector to r1 vector is in the positive right-handed order
  if (Cross<float>(l1_r0, l1_r1) < 0.0) {
    swap(l1_r0, l1_r1);
    swap(l1_p0, l1_p1);
  }
  if (Cross<float>(l2_r0, l2_r1) < 0.0) {
    swap(l2_r0, l2_r1);
    swap(l2_p0, l2_p1);
  }

  if (kDebug) {
    printf(
      "l1_r0:%f,%f l1_r1:%f,%f\nl2_r0:%f,%f l2_r1:%f,%f\n",
      PRINT_VEC2(l1_r0),
      PRINT_VEC2(l1_r1),
      PRINT_VEC2(l2_r0),
      PRINT_VEC2(l2_r1));
  }
  if (((Cross(l1_r0, l2_r0) >= 0.0) && (Cross(l1_r1, l2_r0) >= 0.0) ) ||
    ((Cross(l2_r0, l1_r0) >= 0.0) && (Cross(l2_r1, l1_r0) >= 0.0) ))
  {
    // No Line interaction.
    if (kDebug) {
      printf("No line interaction\n");
    }
    return;
  }

  // The semi-infinite ray from loc and passing through test_line.p0 intersects
  // the trim_line.
  const bool rayOcclusion0 = trim_line.RayIntersects(loc, l1_r0);
  // The semi-infinite ray from loc and passing through test_line.p1 intersects
  // trim_line.
  const bool rayOcclusion1 = trim_line.RayIntersects(loc, l1_r1);

  if (kDebug) {
    printf(
      "rayOcclusion0:%d rayOcclusion1:%d\n",
      rayOcclusion0,
      rayOcclusion1);
  }

  // Vector2f p;
  const bool completeOcclusion =
    test_line.Intersects(loc, l2_p0) && test_line.Intersects(loc, l2_p1);

  // test_line.p0 is in front of, and occludes trim_line.
  const bool occlusion0 = rayOcclusion0 &&
    (trim_line.Touches(loc) || trim_line.Touches(l1_p0) ||
    !trim_line.Intersects(loc, l1_p0));

  // test_line.p1 is in front of, and occludes trim_line.
  const bool occlusion1 = rayOcclusion1 &&
    (trim_line.Touches(loc) || trim_line.Touches(l1_p1) ||
    !trim_line.Intersects(loc, l1_p1));

  if (kDebug) {
    printf(
      "completeOcclusion:%d occlusion0:%d occlusion1:%d\n",
      completeOcclusion,
      occlusion0,
      occlusion1);
    printf(
      "crosses0:%d crosses1:%d\n",
      trim_line.Crosses(loc, l1_p0),
      trim_line.Crosses(loc, l1_p1));
  }
  if (completeOcclusion) {
    if (kDebug) {
      printf("Case 2: trim line completely occluded by test line\n");
    }
    // Trim the line to zero length.
    trim_line.Set(Vector2f(0.1, 0.1), Vector2f(0.1, 0.1));
    return;
  } else if (occlusion0 && occlusion1) {
    if (kDebug) {
      printf("Case 3: partial occlusion in middle\n");
    }
    // trim_line is partially occluded in the middle by test_line. Break up
    // into 2 segments, make trim_line one segment, push back the other
    // segment to the sceneLines list.
    const Vector2f right_section_end = trim_line.RayIntersection(loc, l1_r0);
    const Vector2f left_section_end = trim_line.RayIntersection(loc, l1_r1);
    if (kDebug) {
      printf(
        "Right: %f,%f  Left: %f,%f\n",
        right_section_end.x(),
        right_section_end.y(),
        left_section_end.x(),
        left_section_end.y());
    }
    trim_line.Set(l2_p0, right_section_end);
    // save the unoccluded part of trim_line at its left hand end, if any
    if ((left_section_end - l2_p1).squaredNorm() > sqeps) {
      scene_lines.push_back(Line2f(left_section_end, l2_p1));
    }
  } else if (occlusion0) {
    if (kDebug) {
      printf("Case 5: left end occluded\n");
    }
    // The left hand end of trim_line is occluded, trim it
    Vector2f right_section_end = trim_line.RayIntersection(loc, l1_r0);
    trim_line.Set(l2_p0, right_section_end);
  } else if (occlusion1) {
    if (kDebug) {
      printf("Case 6: right end occluded\n");
    }
    // The right hand end of trim_line is occluded, trim it
    Vector2f left_section_end = trim_line.RayIntersection(loc, l1_r1);
    trim_line.Set(left_section_end, l2_p1);
  } else {
    if (kDebug) {
      printf("Case 7\n");
    }
  }
}


void VectorMap::GetSceneLines(
  const Vector2f & loc,
  float max_range,
  vector<Line2f> * lines_list) const
{
  const float x_min = loc.x() - max_range;
  const float y_min = loc.y() - max_range;
  const float x_max = loc.x() + max_range;
  const float y_max = loc.y() + max_range;
  lines_list->clear();
  for (const Line2f & l : lines) {
    if ((l.p0.x() < x_min) && (l.p1.x() < x_min) ) {
      continue;
    }
    if ((l.p0.y() < y_min) && (l.p1.y() < y_min) ) {
      continue;
    }
    if ((l.p0.x() > x_max) && (l.p1.x() > x_max) ) {
      continue;
    }
    if ((l.p0.y() > y_max) && (l.p1.y() > y_max) ) {
      continue;
    }
    lines_list->push_back(l);
  }
}

void VectorMap::SceneRender(
  const Vector2f & loc,
  float max_range,
  float angle_min,
  float angle_max,
  vector<Line2f> * render) const
{
  static const unsigned int MaxLines = 2000;
  const float eps = Sq(FLAGS_min_line_length);
  vector<Line2f> scene;
  vector<Line2f> lines_list;
  GetSceneLines(loc, max_range, &lines_list);
  render->clear();

  for (size_t i = 0; i < lines_list.size() && i < MaxLines; ++i) {
    Line2f cur_line = lines_list[i];
    // Check if any part of cur_line is unoccluded by present list of lines,
    // as seen from loc.
    for (size_t j = 0; j < scene.size() && cur_line.SqLength() >= eps; ++j) {
      if (scene[j].SqLength() < eps) {
        continue;
      }
      TrimOcclusion(loc, scene[j], &cur_line, &lines_list);
    }

    if (cur_line.SqLength() > eps) {           // At least part of cur_line is unoccluded
      for (size_t j = 0; j < scene.size(); ++j) {
        if (scene[j].SqLength() < eps) {
          continue;
        }
        TrimOcclusion(loc, cur_line, &scene[j], &lines_list);
      }
      // Add the visible part of cur_line.
      scene.push_back(cur_line);
    }
  }

  if (lines_list.size() >= MaxLines) {
    fprintf(
      stderr,
      "Runaway Analytic Scene Render at %.30f,%.30f, %.3f : %.3f\u00b0\n",
      loc.x(), loc.y(),
      RadToDeg(angle_min),
      RadToDeg(angle_max));
  }
  for (const Line2f & l : scene) {
    if (l.SqLength() > eps) {
      render->push_back(l);
    }
  }
}

int GetRayIntersection(
  const Vector2f & loc,
  const size_t skip_line_idx,
  const vector<Line2f> & lines_list,
  Vector2f * ray_end)
{
  Vector2f intersection(0, 0);
  int intersecting_line_idx = -1;
  for (size_t i = 0; i < lines_list.size(); ++i) {
    if (i == skip_line_idx) {
      continue;
    }
    const Line2f & l = lines_list[i];
    if (l.Intersection(loc, *ray_end, &intersection)) {
      *ray_end = intersection;
      intersecting_line_idx = i;
    }
  }
  return intersecting_line_idx;
}

void VectorMap::RayCast(
  const Vector2f & loc,
  float max_range,
  vector<Line2f> * render) const
{
  static const float kEpsilon = 1e-4;

  // Small optimization: ignore all lines not within max_range.
  vector<Line2f> lines_list;
  GetSceneLines(loc, max_range, &lines_list);

  // NOTE(joydeep): In this function, "iidx" refers to the index of
  // the line segment from lines_list that intersects with the associated
  // ray.

  struct RayCastRay
  {
    Vector2f ray_end;
    int iidx;
    RayCastRay(const Vector2f & ray_end, int iidx)
    : ray_end(ray_end),
      iidx(iidx)
    {
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  // Go through all lines, and check for intersection of rays.
  vector<RayCastRay> ray_cast_rays;
  for (size_t i = 0; i < lines_list.size(); ++i) {
    const Line2f & l = lines_list[i];
    const Vector2f dir = kEpsilon * (l.p1 - l.p0).normalized();

    // Add rays from loc to just inside of the line segment.
    Vector2f r0 = l.p0 + dir;
    Vector2f r1 = l.p1 - dir;
    int r0_iidx = GetRayIntersection(loc, i, lines_list, &r0);
    int r1_iidx = GetRayIntersection(loc, i, lines_list, &r1);
    if (r0_iidx < 0) {
      r0_iidx = i;
    }
    if (r1_iidx < 0) {
      r1_iidx = i;
    }
    ray_cast_rays.push_back(RayCastRay(r0, r0_iidx));
    ray_cast_rays.push_back(RayCastRay(r1, r1_iidx));

    // Add rays from loc to max_range just past the line segment.
    Vector2f end_p0 = loc + (l.p0 - dir - loc).normalized() * max_range;
    Vector2f end_p1 = loc + (l.p1 + dir - loc).normalized() * max_range;
    const int end_p0_iidx = GetRayIntersection(loc, i, lines_list, &end_p0);
    const int end_p1_iidx = GetRayIntersection(loc, i, lines_list, &end_p1);
    if (end_p0_iidx >= 0) {
      ray_cast_rays.push_back(RayCastRay(end_p0, end_p0_iidx));
    }
    if (end_p1_iidx >= 0) {
      ray_cast_rays.push_back(RayCastRay(end_p1, end_p1_iidx));
    }
  }

  if (ray_cast_rays.size() < 2) {
    return;
  }
  for (size_t i = 0; i < ray_cast_rays.size(); ++i) {
    if (ray_cast_rays[i].ray_end != ray_cast_rays[i - 1].ray_end) {
      render->push_back(Line2f(loc, ray_cast_rays[i].ray_end));
    }
  }
}


void ShrinkLine(float distance, Line2f * line)
{
  const float len = line->Length();
  const Vector2f dir = line->Dir();
  if (len < 2.0 * distance) {
    return;
  }
  line->p0 += distance * dir;
  line->p1 -= distance * dir;
}

void VectorMap::Cleanup()
{
  const float kShrinkDistance = 1e-4;
  // const float kMinLineLength = 2.0 * kShrinkDistance;
  const float kMinLineLength = 0.05;
  vector<Line2f> new_lines;
  for (size_t i = 0; i < lines.size(); ++i) {
    const Line2f & l1 = lines[i];
    if (l1.Length() < kMinLineLength) {
      continue;
    }
    // Check if l1 intersects with any line in new lines.
    Vector2f p;
    bool intersection = false;
    for (const Line2f l2 : new_lines) {
      if (l2.Intersection(l1, &p)) {
        const Vector2f shrink = kShrinkDistance * l1.Dir();
        const Line2f a = Line2f(l1.p0, p - shrink);
        const Line2f b = Line2f(p + shrink, l1.p1);
        lines.push_back(a);
        lines.push_back(b);
        intersection = true;
        break;
      }
    }
    // No intersection, add it!
    if (!intersection) {
      new_lines.push_back(l1);
    }
  }

  for (Line2f & l : new_lines) {
    ShrinkLine(kShrinkDistance, &l);
  }
  lines = new_lines;
}

void VectorMap::Load(const string & file)
{
  FILE * fid = fopen(file.c_str(), "r");
  if (fid == NULL) {
    fprintf(stderr, "ERROR: Unable to load map %s\n", file.c_str());
    exit(1);
  }
  lines.clear();
  float x1(0), y1(0), x2(0), y2(0);
  while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
    lines.push_back(Line2f(Vector2f(x1, y1), Vector2f(x2, y2)));
  }
  fclose(fid);
  Cleanup();
  file_name = file;
}

bool VectorMap::Intersects(const Vector2f & v0, const Vector2f & v1) const
{
  for (const Line2f & l : lines) {
    if (l.Intersects(v0, v1)) {
      return true;
    }
  }
  return false;
}

void VectorMap::GetPredictedScan(
  const Vector2f & loc,
  float range_min,
  float range_max,
  float angle_min,
  float angle_max,
  int num_rays,
  vector<float> * scan_ptr)
{
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  vector<float> & scan = *scan_ptr;
  vector<Line2f> raycast;
  SceneRender(loc, range_max, angle_min, angle_max, &raycast);
  scan.resize(num_rays);
  std::fill(scan.begin(), scan.end(), range_max);
  if (raycast.empty()) {
    return;
  }
  struct LineCast
  {
    Line2f line;
    float a0;
    float a1;
    bool wraps_around;
  };
  vector<LineCast> line_cast;
  for (size_t i = 0; i < raycast.size(); ++i) {
    const Line2f & r = raycast[i];
    LineCast l;
    l.line.p0 = r.p0 - loc;
    l.line.p1 = r.p1 - loc;
    l.a0 = atan2(l.line.p0.y(), l.line.p0.x());
    l.a1 = atan2(l.line.p1.y(), l.line.p1.x());
    if (fabs(l.a0 - l.a1) < 0.0001) {
      continue;
    }
    l.wraps_around = fabs(l.a1 - l.a0) > M_PI;
    if ((l.wraps_around && (l.a0 < l.a1) ) ||
      (!l.wraps_around && (l.a0 > l.a1) ))
    {
      swap(l.a0, l.a1);
      swap(l.line.p0, l.line.p1);
    }
    line_cast.push_back(l);
  }
  if (line_cast.empty()) {
    return;
  }
  // Iterate over the ray cast, filling the angles
  scan.resize(num_rays);
  const float da = (angle_max - angle_min) / static_cast<float>(num_rays);
  for (int i = 0; i < num_rays; ++i) {
    const float a = AngleMod(angle_min + static_cast<float>(i) * da);
    for (const LineCast & l : line_cast) {
      if ((!l.wraps_around && (l.a0 <= a) && (l.a1 >= a)) ||
        (l.wraps_around && ((l.a0 <= a) || (l.a1 >= a))))
      {
        const Vector2f n = l.line.UnitNormal();
        const Vector2f r(cos(a), sin(a));
        scan[i] = n.dot(l.line.p0) / n.dot(r);
        break;
      }
    }
  }
}

}  // namespace vector_map
