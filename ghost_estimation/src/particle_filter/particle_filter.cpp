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
\file    particle-filter.cpp
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "ghost_estimation/particle_filter/particle_filter.hpp"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

namespace particle_filter {
  Vector2f first_odom_loc;
  float first_odom_angle;

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false){
    }

ParticleFilter::ParticleFilter(ParticleFilterConfig &config_params) :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false){
      config_params_ = config_params;
    }

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  scan.resize((int)(num_ranges / config_params_.resize_factor));
  
  Vector2f sensor_loc = BaseLinkToSensorFrame(loc, angle);
  
  int v_start_index = std::lower_bound(horizontal_lines_.begin(), horizontal_lines_.end(), Line2f(sensor_loc, sensor_loc), horizontal_line_compare) - horizontal_lines_.begin();
  int h_start_index = std::lower_bound(vertical_lines_.begin(), vertical_lines_.end(), Line2f(sensor_loc, sensor_loc), vertical_line_compare) - vertical_lines_.begin();

  // Return if no map is loaded
  if(!vertical_lines_.size() || !horizontal_lines_.size()){
    return;
  }

  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) { // for each ray
    // Initialize the ray line
    float ray_angle = angle + angle_min + config_params_.resize_factor * i / num_ranges * (angle_max - angle_min);
    float C0 = cos(ray_angle);
    float S0 = sin(ray_angle);

    Vector2f final_intersection = sensor_loc + config_params_.range_max * Vector2f(C0, S0);
    Line2f ray(sensor_loc, final_intersection);

    int h_dir = math_util::Sign(ray.Dir().x());
    int v_dir = math_util::Sign(ray.Dir().y());

    std::size_t v_search_index = v_start_index;
    std::size_t h_search_index = h_start_index;

    if(h_dir < 0){
      h_search_index -= 1;
    }

    if(v_dir < 0){
      v_search_index -= 1;
    }

    // Clip bounds so that we don't accidently generate invalid index if we go outside map
    h_search_index = std::min(std::max(h_search_index, std::size_t(0)), vertical_lines_.size());
    v_search_index = std::min(std::max(v_search_index, std::size_t(0)), horizontal_lines_.size());

    Vector2f final_intersection_xy = final_intersection;
    bool intersection_found = false;
    double curr_dist = 0;
    while(!intersection_found && curr_dist < ray.Length()){
      float xi = (h_search_index < vertical_lines_.size()) ? abs(vertical_lines_[h_search_index].p0.x() - ray.p0.x()) : 100;
      float yi = (v_search_index < horizontal_lines_.size()) ? abs(horizontal_lines_[v_search_index].p0.y() - ray.p0.y()) : 100;

      float rx = abs(xi / C0);
      float ry = abs(yi / S0);

        if(rx < ry && rx < ray.Length()){
          curr_dist = rx;
          intersection_found = ray.Intersection(vertical_lines_[h_search_index], &final_intersection_xy);
          h_search_index += h_dir;
        }
        else if(ry <= rx && ry < ray.Length()){
          curr_dist = ry;
          intersection_found = ray.Intersection(horizontal_lines_[v_search_index], &final_intersection_xy);
          v_search_index += v_dir;
        }
        else{
          final_intersection_xy = final_intersection;
          break;
        }
    }

    float curr_dist_angled = range_max;
    Vector2f final_intersection_angled = final_intersection;
    for (size_t i = 0; i < angled_lines_.size(); ++i) {
      if(ray.Intersection(angled_lines_[i], &final_intersection_angled)){
        float new_dist = (final_intersection_angled - ray.p0).norm();
        if(new_dist < curr_dist_angled){
          curr_dist_angled = new_dist;
        }
      }     
    }

    if((final_intersection_angled - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_angled;
    }
    if((final_intersection_xy - ray.p0).norm() < (final_intersection - ray.p0).norm()){
      final_intersection = final_intersection_xy;
    }

    scan[i] = final_intersection;
  }
}

double ParticleFilter::GetRobustObservationLikelihood(double measured, double expected, double dist_short, double dist_long){
  
  if(measured < config_params_.range_min || measured > config_params_.range_max){
    return 0;
  }
  else if(measured < (expected - dist_short)){
    return dist_short;
  }
  else if(measured > (expected + dist_long)){
    return dist_long;
  }
  else{
    return measured - expected;
  }
}

// Update the weight of the particle based on how well it fits the observation
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  
  // Get predicted point cloud
  Particle &particle = *p_ptr;
  vector<Vector2f> predicted_cloud; // map frame

  GetPredictedPointCloud(particle.loc, 
                         particle.angle, 
                         ranges.size(), 
                         range_min, 
                         range_max,
                         angle_min,
                         angle_max,
                         &predicted_cloud);
  Vector2f sensor_loc = BaseLinkToSensorFrame(particle.loc, particle.angle);
  // resize the ranges
  vector<float> trimmed_ranges(predicted_cloud.size());
  particle.weight = 0;

  // Calculate the particle weight
  for(std::size_t i = 0; i < predicted_cloud.size(); i++) {
    int laser_index = i * config_params_.resize_factor;
    if(!config_params_.use_skip_range || laser_index < config_params_.skip_index_min || laser_index > config_params_.skip_index_max){
      trimmed_ranges[i] = ranges[laser_index];
      double predicted_range = (predicted_cloud[i] - sensor_loc).norm();
      double diff = GetRobustObservationLikelihood(trimmed_ranges[i], predicted_range, config_params_.dist_short, config_params_.dist_long);
      particle.weight += -config_params_.gamma * Sq(diff) / Sq(config_params_.sigma_observation);
    }
  } 
}

void ParticleFilter::Resample() {
  vector<Particle> new_particles(particles_.size());
  vector<double> weight_bins(particles_.size());
  
  // Calculate weight sum, get bins sized by particle weights as vector
  double weight_sum = 0;
  for(std::size_t i = 0; i < particles_.size(); i++){
    weight_sum += particles_[i].weight;
    weight_bins[i] = weight_sum;
  }

  // During resampling:
  for(std::size_t i = 0; i < particles_.size(); i++){
    double rand_weight = rng_.UniformRandom(0, weight_sum);
    auto new_particle_index = std::lower_bound(weight_bins.begin(), weight_bins.end(), rand_weight) - weight_bins.begin();
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size());
  }
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::LowVarianceResample() {
  vector<Particle> new_particles(particles_.size());

  double select_weight = rng_.UniformRandom(0, weight_sum_);

  for(std::size_t i = 0; i < particles_.size(); i++){
    int new_particle_index = std::lower_bound(weight_bins_.begin(), weight_bins_.end(), select_weight) - weight_bins_.begin();
    select_weight = std::fmod(select_weight + weight_sum_/((double) particles_.size()), weight_sum_);
    new_particles[i] = particles_[new_particle_index];
    new_particles[i].weight = 1/((double) particles_.size()); // rng_.UniformRandom(); good for testing
  }
  weight_sum_ = 1.0;
  
  // After resampling:
  particles_ = new_particles;
}

void ParticleFilter::SetParticlesForTesting(vector<Particle> new_particles){
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  double delta_translation = (last_update_loc_ - prev_odom_loc_).norm();
  double delta_angle = math_util::AngleDiff(last_update_angle_, prev_odom_angle_);
  if(delta_translation > config_params_.min_update_dist || std::abs(delta_angle) > config_params_.min_update_angle){
    static int i = 0;
    double start_time = GetMonotonicTime();
    max_weight_log_ = -1e10; // Should be smaller than any
    weight_sum_ = 0;
    weight_bins_.resize(particles_.size());
    std::fill(weight_bins_.begin(), weight_bins_.end(), 0);

    // Update each particle with log error weight and find largest weight (smallest negative number)
    double particle_update_start = GetMonotonicTime();
    double p_update_start = 0;
    double p_update_diff_avg = 0;
    for(Particle &p: particles_){
      p_update_start = GetMonotonicTime();
      Update(ranges, range_min, range_max, angle_min, angle_max, &p);
      max_weight_log_ = std::max(max_weight_log_, p.weight);
      p_update_diff_avg += 1000000*(GetMonotonicTime() - p_update_start);
    }
    double particle_update_diff = 1000*(GetMonotonicTime() - particle_update_start);
    p_update_diff_avg /= particles_.size();
    // std::cout << "Particle Update (ms): " << particle_update_diff << " Avg (us): " << p_update_diff_avg << std::endl;

    // Normalize log-likelihood weights by max log weight and transform back to linear scale
    // Sum all linear weights and generate bins
    for(std::size_t i = 0; i < particles_.size(); i++){
      particles_[i].weight = exp(particles_[i].weight - max_weight_log_);
      weight_sum_ += particles_[i].weight;
      weight_bins_[i] = weight_sum_;
    }

    if(!(resample_loop_counter_ % config_params_.resample_frequency)){
      LowVarianceResample();
    }
    last_update_loc_ = prev_odom_loc_;
    last_update_angle_ = prev_odom_angle_;
    resample_loop_counter_++;

    end_time += 1000*(GetMonotonicTime() - start_time);
    if(i%10 == 0){
      std::cout << "Total Update Avg (ms): " << end_time/10.0 << std::endl << std::endl;;
      end_time = 0;
    }
    i++;
  }                     
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // propagate particles forward based on odometry.

  // rotation matrix from last odom to last baselink
  auto rot_odom1_to_bl1 = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  
  // Change in translation and angle from odometry
  Eigen::Vector2f delta_translation = rot_odom1_to_bl1 * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  // Get translation noise in Base Link 2
  float sigma_x =
    config_params_.k1 * delta_translation.x() +
    config_params_.k2 * delta_translation.y() +
    config_params_.k3 * abs(delta_angle);
  float sigma_y =
    config_params_.k4 * delta_translation.x() +
    config_params_.k5 * delta_translation.y() +
    config_params_.k6 * abs(delta_angle);
  // Get noisy angle
  float sigma_tht =
    config_params_.k7 * delta_translation.x() +
    config_params_.k8 * delta_translation.y() +
    config_params_.k9 * abs(delta_angle);

  for(Particle &particle: particles_){
    Eigen::Vector2f e_xy = Eigen::Vector2f((float) rng_.Gaussian(0.0, sigma_x),(float) rng_.Gaussian(0.0, sigma_y));
    Eigen::Vector2f noisy_translation = delta_translation + e_xy;
    float noisy_angle = delta_angle + rng_.Gaussian(0.0, sigma_tht);
    
    // Transform noise to map using current particle angle
    auto rot_bl1_to_map = Eigen::Rotation2D<float>(particle.angle).toRotationMatrix();
    particle.loc += rot_bl1_to_map * noisy_translation;   
    particle.angle += noisy_angle;        
  }

  // Update previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  // double d_translation = (last_update_loc_ - prev_odom_loc_).norm();
  // double d_angle = math_util::AngleDiff(last_update_angle_, prev_odom_angle_);

  // std::cout << "Motion Model: " << d_translation << ", " << d_angle << std::endl;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log.

  particles_.resize(config_params_.num_particles);
  std::cout << "Num Particles: " << config_params_.num_particles << std::endl;

  for(Particle &particle: particles_){
    particle.loc = Eigen::Vector2f(
      loc[0] + rng_.Gaussian(0, config_params_.init_x_sigma),
      loc[1] + rng_.Gaussian(0, config_params_.init_y_sigma)
      );
    particle.angle = angle + rng_.Gaussian(0, config_params_.init_r_sigma);
    particle.weight = 1/((double)particles_.size());
  }
  weight_sum_ = 1;
  max_weight_log_ = 0;
  last_update_loc_ = prev_odom_loc_;
  last_update_angle_ = prev_odom_angle_;
  map_.Load(map_file);
  SortMap();
}

bool ParticleFilter::horizontal_line_compare(const geometry::Line2f l1, const geometry::Line2f l2){
  return l1.p0.y() < l2.p0.y();
}

bool ParticleFilter::vertical_line_compare(const geometry::Line2f l1, const geometry::Line2f l2){
  return l1.p0.x() < l2.p0.x();
}

void ParticleFilter::SortMap(){
   // Split lines in map into horizontal, vertical, and angled
  horizontal_lines_.clear();
  vertical_lines_.clear();
  angled_lines_.clear();
  
  for (size_t i = 0; i < map_.lines.size(); ++i) {
      const geometry::Line2f line = map_.lines[i];
      if(line.p0.y() == line.p1.y()){
        horizontal_lines_.push_back(line);
      }
      else if(line.p0.x() == line.p1.x()){
        vertical_lines_.push_back(line);
      }
      else{
        angled_lines_.push_back(line);
      }
  }
  // Sort horizontal and vertical in ascending order
  std::sort(horizontal_lines_.begin(), horizontal_lines_.end(), horizontal_line_compare);
  std::sort(vertical_lines_.begin(), vertical_lines_.end(), vertical_line_compare);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles.

  Eigen::Vector2f angle_point = Eigen::Vector2f(0, 0);
  for(Particle particle: particles_){
    loc += particle.loc * particle.weight;
    angle_point += Eigen::Vector2f(cos(particle.angle), sin(particle.angle)) * particle.weight;
  }

  loc /= weight_sum_;
  angle_point /= weight_sum_;
  angle = atan2(angle_point[1], angle_point[0]);
}

Eigen::Vector2f ParticleFilter::BaseLinkToSensorFrame(const Eigen::Vector2f &loc, const float &angle){
  return loc + Vector2f(config_params_.laser_offset*cos(angle), config_params_.laser_offset*sin(angle));
}

}  // namespace particle_filter