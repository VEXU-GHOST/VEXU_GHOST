//========================================================================
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
//========================================================================
/*!
\file    vector_map.h
\brief   Vector map representation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "math/line2d.h"

#ifndef VECTOR_MAP_H
#define VECTOR_MAP_H

namespace vector_map {

// Checks if any part of trim_line is occluded by test_line when seen from
// loc, and if so, trim_line is trimmed accordingly, adding sub-lines to
// scene_lines if necessary.
void TrimOcclusion(const Eigen::Vector2f& loc,
                  const geometry::line2f& line1,
                  geometry::line2f* line2_ptr,
                  std::vector<geometry::line2f>* scene_lines_ptr);

struct VectorMap {
  VectorMap() {}
  explicit VectorMap(const std::vector<geometry::line2f>& lines) :
      lines(lines) {}
  explicit VectorMap(const std::string& file) {
    Load(file);
  }

  void GetSceneLines(const Eigen::Vector2f& loc,
                     float max_range,
                     std::vector<geometry::line2f>* lines_list) const;


  void SceneRender(const Eigen::Vector2f& loc,
                   float max_range,
                   float angle_min,
                   float angle_max,
                   std::vector<geometry::line2f>* render) const;

  void RayCast(const Eigen::Vector2f& loc,
               float max_range,
               std::vector<geometry::line2f>* render) const;

  // Get predicted laser scan from current location.
  void GetPredictedScan(const Eigen::Vector2f& loc,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        int num_rays,
                        std::vector<float>* scan);
  void Cleanup();

  void Load(const std::string& file);

  bool Intersects(const Eigen::Vector2f& v0, const Eigen::Vector2f& v1) const ;
  std::vector<geometry::line2f> lines;
  std::string file_name;
};



}  // namespace vector_map

#endif  // VECTOR_MAP_H