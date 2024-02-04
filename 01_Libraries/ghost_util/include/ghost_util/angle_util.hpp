#pragma once

#include <cmath>

namespace ghost_util {

float WrapAngle360(float angle);
float WrapAngle180(float angle);
float FlipAngle180(float angle);

float WrapAngle2PI(float angle);
float WrapAnglePI(float angle);
float FlipAnglePI(float angle);

float SmallestAngleDistDeg(float a2, float a1);
float SmallestAngleDistRad(float a2, float a1);

} // namespace ghost_util