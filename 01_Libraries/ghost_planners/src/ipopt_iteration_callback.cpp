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

#include "ghost_planners/ipopt_iteration_callback.hpp"

using namespace casadi;

namespace ghost_planners
{

IterationCallback::IterationCallback(
  int nx, int ng, std::shared_ptr<std::deque<IPOPTOutput>> data_buffer,
  std::shared_ptr<std::mutex> data_mutex,
  std::shared_ptr<std::atomic_bool> exit_flag_ptr)
: nx_(nx),
  ng_(ng),
  data_buffer_(data_buffer),
  data_mutex_(data_mutex),
  exit_flag_ptr_(exit_flag_ptr),
  casadi::Callback()
{
  construct("iteration_callback");
}

bool IterationCallback::has_eval_buffer() const
{
  return true;
}

void IterationCallback::init()
{
}

casadi_int IterationCallback::get_n_in()
{
  return nlpsol_n_out();
}

casadi_int IterationCallback::get_n_out()
{
  return 1;
}

std::string IterationCallback::get_name_in(casadi_int i)
{
  return nlpsol_out(i);
}

Sparsity IterationCallback::get_sparsity_in(casadi_int i)
{
  auto n = nlpsol_out(i);
  if (n == "f") {
    return Sparsity::scalar();
  } else if ((n == "x") || (n == "lam_x") ) {
    return Sparsity::dense(nx_);
  } else if ((n == "g") || (n == "lam_g") ) {
    return Sparsity::dense(ng_);
  } else {
    return Sparsity(0, 0);
  }
}

std::string IterationCallback::get_name_out(casadi_int i)
{
  return "ret";
}

std::vector<DM> IterationCallback::eval(const std::vector<DM> & arg) const
{
  return {0};
}

int IterationCallback::eval_buffer(
  const double ** arg, const std::vector<casadi_int> & sizes_arg,
  double ** res, const std::vector<casadi_int> & sizes_res) const
{
  auto output_map = nlpsol_out();
  IPOPTOutput data{};

  data.state_vector.resize(nx_);
  data.constraint_vector.resize(ng_);
  data.state_lagrange_multipliers.resize(nx_);
  data.constraint_lagrange_multipliers.resize(ng_);

  for (int i = 0; i < sizes_arg[0]; i++) {
    data.state_vector[i] = arg[0][i];
  }
  data.cost = arg[1][0];

  for (int i = 0; i < sizes_arg[2]; i++) {
    data.constraint_vector[i] = arg[2][i];
  }

  for (int i = 0; i < sizes_arg[3]; i++) {
    data.state_lagrange_multipliers[i] = arg[3][i];
  }

  for (int i = 0; i < sizes_arg[4]; i++) {
    data.constraint_lagrange_multipliers[i] = arg[4][i];
  }

  data.iteration = iteration_count_;

  std::unique_lock<std::mutex> lock(*data_mutex_);
  data_buffer_->push_front(data);
  iteration_count_++;

  if (*exit_flag_ptr_) {
    *(res[0]) = 1.0;
    *exit_flag_ptr_ = false;
  }
  return {0};
}

} // namespace ghost_planners
