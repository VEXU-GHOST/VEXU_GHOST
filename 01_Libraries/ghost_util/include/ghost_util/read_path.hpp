#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "eigen3/Eigen/Dense"
#include "ghost_util/unit_conversion_utils.hpp"

constexpr double WALL_WIDTH_CM = 2 * 2.54;
constexpr double FIELD_WIDTH_CM = 12 * 12 * 2.54;
constexpr double FIELD_MAX =  (FIELD_WIDTH_CM / 2) - WALL_WIDTH_CM;
constexpr double FIELD_MIN = -(FIELD_WIDTH_CM / 2) + WALL_WIDTH_CM;
constexpr double CM_TO_TILES = 1 / (24 * 2.54);
constexpr double TILES_TO_METERS = 0.6096;

namespace ghost_util {

std::vector<std::vector<double>> readPathFromFile(const std::string &filename) {

    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open " + filename + ".");
    }

    std::vector<double> x_trajectory;
    std::vector<double> y_trajectory;
    std::vector<double> theta_trajectory;

    std::string line;
    double current_theta = 0;
    while (std::getline(file, line)) {

        std::stringstream ss(line);
        std::string value_string;
        double value;

        // Ignore metadata
        if (ss.peek() == '#') {
            continue;
        }

        // Get Y setpoint
        std::getline(ss, value_string, ',');
        y_trajectory.push_back(
            // 6.0 * TILES_TO_METERS - (std::clamp((std::stod(value_string)), FIELD_MIN, FIELD_MAX) + FIELD_WIDTH_CM / 2) / 100.0
            (std::clamp((std::stod(value_string)), FIELD_MIN, FIELD_MAX) + FIELD_WIDTH_CM / 2) / 100.0
        );
        
        // Get X setpoint
        std::getline(ss, value_string, ',');
        x_trajectory.push_back(
            6.0 * TILES_TO_METERS - (std::clamp((std::stod(value_string)), FIELD_MIN, FIELD_MAX) + FIELD_WIDTH_CM / 2) / 100.0
        );

        // Ignore speed
        std::getline(ss, value_string, ',');
        value_string = "";

        // Get Theta setpoint
        std::getline(ss, value_string, ',');
        if (value_string != "") {
            current_theta = (-std::stod(value_string) * DEG_TO_RAD + M_PI);
            if (current_theta > 360.0){
                current_theta -= 360.0;
            } else if (current_theta < 0.0){
                current_theta += 360;
            }
        }
        theta_trajectory.push_back(current_theta);
    }

    // Close the file
    file.close();

    return {x_trajectory, y_trajectory, theta_trajectory};
}

}  // namespace ghost_util