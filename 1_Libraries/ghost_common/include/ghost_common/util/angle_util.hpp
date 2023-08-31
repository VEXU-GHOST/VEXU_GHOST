#pragma once

#include <cmath>

namespace ghost_common {

const float DEG_TO_RAD = M_PI / 180.0;
const float RAD_TO_DEG = 180.0 / M_PI;

float WrapAngle360(float angle);
float WrapAngle180(float angle);
float FlipAngle180(float angle);

float WrapAngle2PI(float angle);
float WrapAnglePI(float angle);
float FlipAnglePI(float angle);

float SmallestAngleDistDeg(float a2, float a1);
float SmallestAngleDistRad(float a2, float a1);

}