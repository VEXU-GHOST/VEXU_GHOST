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

#pragma once

#include <atomic>
#include <deque>
#include <vector>
#include <mutex>
#include <memory>

#include <casadi/casadi.hpp>

using namespace casadi;

namespace ghost_planners
{
class IterationCallback : public casadi::Callback
{
public:
  struct IPOPTOutput
  {
    IPOPTOutput()
    {
    }

    IPOPTOutput(const IPOPTOutput & rhs)
    {
      state_vector = rhs.state_vector;
      constraint_vector = rhs.constraint_vector;
      cost = rhs.cost;
      state_lagrange_multipliers = rhs.state_lagrange_multipliers;
      constraint_lagrange_multipliers = rhs.constraint_lagrange_multipliers;
      iteration = rhs.iteration;
    }

    bool operator==(const IPOPTOutput & rhs) const
    {
      bool eq = true;
      eq &= (iteration == rhs.iteration);
      eq &= (std::fabs(cost - rhs.cost) < 2 * std::numeric_limits<double>::epsilon());
      eq &= (state_vector == rhs.state_vector);
      eq &= (constraint_vector == rhs.constraint_vector);
      eq &= (state_lagrange_multipliers == rhs.state_lagrange_multipliers);
      eq &= (constraint_lagrange_multipliers == rhs.constraint_lagrange_multipliers);
      return eq;
    }

    int iteration;
    double cost = 0.0;
    std::vector<double> state_vector;
    std::vector<double> constraint_vector;
    std::vector<double> state_lagrange_multipliers;
    std::vector<double> constraint_lagrange_multipliers;
  };

  IterationCallback(
    int nx, int ng, std::shared_ptr<std::deque<IPOPTOutput>> data_buffer,
    std::shared_ptr<std::mutex> data_mutex,
    std::shared_ptr<std::atomic_bool> exit_flag_ptr);

  bool has_eval_buffer() const override;

  void init() override;

  casadi_int get_n_in() override;

  casadi_int get_n_out() override;

  std::string get_name_in(casadi_int i) override;

  Sparsity get_sparsity_in(casadi_int i) override;

  std::string get_name_out(casadi_int i) override;
  std::vector<DM> eval(const std::vector<DM> & arg) const override;

  /**
   * This is effectively the only method override that matters if you want to extend this class.
   */
  int eval_buffer(
    const double ** arg, const std::vector<casadi_int> & sizes_arg,
    double ** res, const std::vector<casadi_int> & sizes_res) const override;

  int nx_;
  int ng_;
  std::shared_ptr<std::deque<IPOPTOutput>> data_buffer_;
  std::shared_ptr<std::mutex> data_mutex_;
  std::shared_ptr<std::atomic_bool> exit_flag_ptr_;
  mutable int iteration_count_ = 1;
};

} // namespace ghost_planners
