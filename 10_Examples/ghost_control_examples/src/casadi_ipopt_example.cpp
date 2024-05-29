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

#include <map>
#include <casadi/casadi.hpp>

using namespace casadi;

int main(int argc, char * argv[])
{
  // Define list/vector of optimization variables
  auto opt_vars = SX::zeros(2);
  opt_vars(0) = SX::sym("x");
  opt_vars(1) = SX::sym("y");

  // Define cost function as f(x, y) = (x-y)^2 + x^2
  auto f = SX::zeros(1);
  f = (opt_vars(0) - opt_vars(1)) * (opt_vars(0) - opt_vars(1)) + opt_vars(0) * opt_vars(0);

  // Setup bounds and constraints
  auto constraint_vector = SX::zeros(1);
  constraint_vector(0) = opt_vars(0) * opt_vars(0);       // x^2 >= 10

  // Set bounds for x and y, where y <= 2 and x is unbounded.
  auto xlb = DM::zeros(2);
  auto xub = DM::zeros(2);
  xlb(0) = -DM::inf();
  xub(0) = DM::inf();
  xlb(1) = -DM::inf();
  xub(1) = 2;

  // Set initial guess for x and y to 1
  auto x0 = DM::ones(2);

  /////////////////////////
  ///// Create Solver /////
  /////////////////////////
  SXDict nlp_config{
    {"x", opt_vars},
    {"f", f},
    {"g", constraint_vector}};
  Dict solver_config{
    // {"verbose", true}
  };
  auto solver = nlpsol("solver", "ipopt", nlp_config, solver_config);

  //////////////////////
  ///// Test Solve /////
  //////////////////////
  std::map<std::string, DM> solver_args, res;
  solver_args["lbx"] = xlb;
  solver_args["ubx"] = xub;
  solver_args["lbg"] = 10;
  solver_args["ubg"] = DM::inf();
  solver_args["x0"] = x0;
  res = solver(solver_args);

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "Final values for x and y are :" << res.at("x") << std::endl;

  return 0;
}
