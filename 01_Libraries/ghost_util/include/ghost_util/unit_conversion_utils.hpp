#pragma once

#include <math.h>

namespace ghost_util {

// Common conversions
inline constexpr double RPM_TO_RAD_PER_SEC = M_PI / 30.0;
inline constexpr double RAD_PER_SEC_TO_RPM = 30.0 / M_PI;
inline constexpr double RAD_TO_DEG = 180.0 / M_PI;
inline constexpr double DEG_TO_RAD = M_PI / 180.0;

inline constexpr double INCHES_TO_METERS = 2.54 / 100.0;
inline constexpr double METERS_TO_INCHES = 100.0 / 2.54;

} // namespace ghost_util