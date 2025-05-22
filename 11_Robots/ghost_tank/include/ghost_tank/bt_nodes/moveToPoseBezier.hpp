/*
*   Copyright (c) 2024 Jake Wendling
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

#pragma once
#include "ghost_planners/bezier_curve.hpp"
#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/moveToPose.hpp"

using std::placeholders::_1;
using ghost_planners::BezierCurve;

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
class MoveToPoseBezier : public MoveToPose
{
public:
  // If your Node has ports, you must use this constructor signature
  MoveToPoseBezier(const std::string & name, const BT::NodeConfig & config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<BezierCurve> bezier_;

  double posX_m{0.0};
  double posY_m{0.0};
  double theta_rad{0.0};
  double lead{0.0};

  // gets all member variables from ports, must deal with mirrored also 
  void GetBlackboardData();
  void FirstLoop();
  void GeneratePath();
};

} // namespace ghost_tank {
