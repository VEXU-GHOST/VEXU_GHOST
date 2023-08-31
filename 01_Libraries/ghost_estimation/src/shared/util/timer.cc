// Copyright 2016-2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include "util/timer.h"

#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <string>

using std::max;
using std::string;

#if defined(__i386__)
uint64_t RDTSC() {
  uint64_t x;
  __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
  return x;
}
#elif defined(__x86_64__)
uint64_t RDTSC() {
  uint32_t hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  const uint64_t x =
      static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32);
  return x;
}
#endif

double GetWallTime() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  const double time =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec)*(1.0E-9);
  return time;
}

double GetMonotonicTime() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const double time =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec)*(1.0E-9);
  return time;
}

void Sleep(double duration) {
  const useconds_t duration_usec = static_cast<useconds_t>(duration * 1.0E6);
  usleep(duration_usec);
}

RateLoop::RateLoop(double rate) :
    t_last_run_(0.0), delay_interval_(1.0 / rate) { }

void RateLoop::Sleep() {
  const double t_now = GetMonotonicTime();
  const double sleep_duration = max(0.0, delay_interval_ + t_last_run_ - t_now);
  ::Sleep(sleep_duration);
  t_last_run_ = t_now + sleep_duration;
}

FunctionTimer::FunctionTimer(const char* name) :
    name_(name), t_start_(GetMonotonicTime()), t_lap_start_(t_start_) {}

FunctionTimer::~FunctionTimer() {
  const double t_stop = GetMonotonicTime();
  printf("%s: %f ms\n", name_.c_str(), 1.0E3 * (t_stop - t_start_));
}

void FunctionTimer::Lap(int id) {
  const double t_now = GetMonotonicTime();
  printf("%s(%d): %f ms\n",
         name_.c_str(),
         id,
         1.0E3 * (t_now - t_lap_start_));
  t_lap_start_ = t_now;
}

CumulativeFunctionTimer::Invocation::Invocation(
    CumulativeFunctionTimer* cumulative_timer) :
    t_start_(GetMonotonicTime()), cumulative_timer_(cumulative_timer) {}

CumulativeFunctionTimer::Invocation::~Invocation() {
  const double t_duration = GetMonotonicTime() - t_start_;
  cumulative_timer_->total_invocations_++;
  cumulative_timer_->total_run_time_ += t_duration;
}

CumulativeFunctionTimer::CumulativeFunctionTimer(const char* name) :
    name_(name), total_run_time_(0.0), total_invocations_(0) {}

CumulativeFunctionTimer::~CumulativeFunctionTimer() {
  const double mean_run_time =
      total_run_time_ / static_cast<double>(total_invocations_);
  printf("Run-time stats for %s : mean run time = %f ms, "
         "invocations = %" PRIu64 "\n",
         name_.c_str(),
         1.0E3 * mean_run_time,
         total_invocations_);
}


RateChecker::Invocation::Invocation(
    RateChecker* rate_checker) :
    t_start_(GetMonotonicTime()), rate_checker_(rate_checker) {
  rate_checker_->total_invocations_++;
  if (rate_checker_-> t_last_run_ > 0.0) {
    rate_checker_->total_intervals_ += (t_start_ - rate_checker_->t_last_run_);
  }
  rate_checker_->t_last_run_ = t_start_;
}

RateChecker::Invocation::~Invocation() {
  const double t_duration = GetMonotonicTime() - t_start_;
  rate_checker_->total_run_time_ += t_duration;
}

RateChecker::RateChecker(const char* name) :
    name_(name),
    t_last_run_(0.0),
    total_intervals_(0),
    total_run_time_(0.0),
    total_invocations_(0) {}

RateChecker::~RateChecker() {
  const double mean_run_time =
      total_run_time_ / static_cast<double>(total_invocations_);
  const double mean_interval =
      total_intervals_ / static_cast<double>(total_invocations_);
  printf("Run-time stats for %s : mean run time = %f ms, "
         "invocations = %" PRIu64 ", mean interval = %f ms\n",
         name_.c_str(),
         1.0E3 * mean_run_time,
         total_invocations_,
         1.0E3 * mean_interval);
}