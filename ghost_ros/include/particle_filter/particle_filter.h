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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  void LowVarianceResample();

  void SortMap();
  static bool horizontal_line_compare(const geometry::line2f l1, const geometry::line2f l2);
  static bool vertical_line_compare(const geometry::line2f l1, const geometry::line2f l2);

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

  void SetParticlesForTesting(std::vector<Particle> new_particles);

 Eigen::Vector2f BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle);

 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;
  std::vector<geometry::line2f> horizontal_lines_;
  std::vector<geometry::line2f> vertical_lines_;
  std::vector<geometry::line2f> angled_lines_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Eigen::Vector2f first_odom_loc;
  // float first_odom_angle;
  // bool first_odom_flag = true;
  
  std::vector<double> weight_bins_;
  double max_weight_log_ = 0;
  double weight_sum_ = 0;

  Eigen::Vector2f last_update_loc_;
  float last_update_angle_;
  int resample_loop_counter_ = 0;


  double end_time = 0;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_