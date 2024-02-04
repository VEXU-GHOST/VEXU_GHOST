#pragma once

#include <cmath>

namespace ghost_util {

double WrapAngle360(double angle);
double WrapAngle180(double angle);
double FlipAngle180(double angle);

double WrapAngle2PI(double angle);
double WrapAnglePI(double angle);
double FlipAnglePI(double angle);

double SmallestAngleDistDeg(double a2, double a1);
double SmallestAngleDistRad(double a2, double a1);

} // namespace ghost_util