/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"

namespace ghost_util
{

double WrapAngle360(double angle)
{
  angle = fmod(angle, 360.0);

  if (angle < 0.0) {
    angle += 360.0;
  }

  return angle;
}

double WrapAngle180(double angle)
{
  angle = WrapAngle360(angle);

  if (angle > 180.0) {
    angle -= 360.0;
  }

  return angle;
}

double WrapAngle2PI(double angle)
{
  return DEG_TO_RAD * WrapAngle360(angle * RAD_TO_DEG);
}

double WrapAnglePI(double angle)
{
  return DEG_TO_RAD * WrapAngle180(angle * RAD_TO_DEG);
}

double FlipAngle180(double angle)
{
  return WrapAngle180(angle + 180.0);
}

double FlipAnglePI(double angle)
{
  return WrapAnglePI(angle + M_PI);
}

double SmallestAngleDistDeg(double a2, double a1)
{
  a1 = WrapAngle360(a1);
  a2 = WrapAngle360(a2);

  double diff = a2 - a1;
  if (diff > 180.0) {
    diff = -(360.0 - diff);
  } else if (diff < -180.0) {
    diff = 360.0 + diff;
  }
  return diff;
}

double SmallestAngleDistRad(double a2, double a1)
{
  return DEG_TO_RAD * SmallestAngleDistDeg(RAD_TO_DEG * a2, RAD_TO_DEG * a1);
}

double quaternionToYawDeg(double w, double x, double y, double z)
{
  return quaternionToYawRad(w, x, y, z) * RAD_TO_DEG;
}
void yawToQuaternionDeg(double yaw, double & w, double & x, double & y, double & z)
{
  yawToQuaternionRad(yaw * DEG_TO_RAD, w, x, y, z);
}

double quaternionToYawRad(double w, double x, double y, double z)
{
  return 2.0 * atan2(z, w);
}
void yawToQuaternionRad(double yaw, double & w, double & x, double & y, double & z)
{
  w = cos(yaw * 0.5);
  x = 0.0;
  y = 0.0;
  z = sin(yaw * 0.5);
}

} // namespace ghost_util
