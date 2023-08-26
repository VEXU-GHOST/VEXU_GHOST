// Copyright 2016 - 2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Timing and profiling library.
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

#include <stdint.h>

#include <string>

#ifndef SRC_UTIL_TIMER_H_
#define SRC_UTIL_TIMER_H_

// Helper class to execute tightly timed loops.
class RateLoop {
 public:
  // Primary constructor, initialize a fixed rate.
  explicit RateLoop(double rate);

  // Sleep for as long as necessary to run at the specified rate.
  void Sleep();

 private:
  // Disable default constructor.
  RateLoop();

 private:
  double t_last_run_;
  const double delay_interval_;
};

// Return the value of the CPU TSC register.
uint64_t RDTSC();

// Get wall time in seconds elapsed since epoch.
double GetWallTime();

// Get a high-resolution monotonic clock time in seconds. This is useful
// for code profiling.
double GetMonotonicTime();

// Sleep for the specified duration in seconds.
void Sleep(double duration);

// Timer to profile function time execution. To use the timer, place the
// following statement at the begining of the function:
// FunctionTimer ft(__PRETTY_FUNCTION__);
class FunctionTimer {
 public:
  // Primary constructor, pass in the function name, or whatever label you wish
  // to identify the timer by.
  explicit FunctionTimer(const char* name);

  // Default destructor. Stop the timer, and print the time duration between the
  // creation and destruction of the timer.
  ~FunctionTimer();

  // Lap the timer: print the time elapsed since the last lap, with a user-
  // provided lap ID, usually the line number from where the lap was called.
  void Lap(int id);

 private:
  // Disable copy constructor.
  FunctionTimer(const FunctionTimer&);
  // Disable default constructor.
  FunctionTimer();

 private:
  // Name of the timer.
  const std::string name_;
  // Start time.
  const double t_start_;
  // Last lap start time.
  double t_lap_start_;
};

// Timer to profile repeated invocations of a function. To use this timer,
// declare an instance of CumulativeFunctionTimer at a higher scope than the
// function, and instantiate an Invocation for each function.
// Example:
// ==============================
// CumulativeFunctionTimer foo_function_timer_("Foo");
// void Foo() {
//   CumulativeFunctionTimer::Invocation invoke(&foo_function_timer_);
//   // ... Do some stuff ...
// }
// ==============================
class CumulativeFunctionTimer {
 public:
  class Invocation {
   public:
    // Primary constructor, starts the timer.
    explicit Invocation(CumulativeFunctionTimer* cumulative_timer);

    // Default destructor, Stops the timer.
    ~Invocation();
   private:
    // Disable copy constructor.
    Invocation(const Invocation&);
    // Disable default constructor.
    Invocation();

   private:
    // Start time.
    const double t_start_;
    // Pointer to cumulative timer.
    CumulativeFunctionTimer* const cumulative_timer_;
  };

 public:
  // Primary constructor, pass in the function name, or whatever label you wish
  // to identify the timer by.
  explicit CumulativeFunctionTimer(const char* name);

  // Default destructor. Print statistics of all invocations.
  ~CumulativeFunctionTimer();

 private:
  // Disable copy constructor.
  CumulativeFunctionTimer(const CumulativeFunctionTimer&);
  // Disable default constructor.
  CumulativeFunctionTimer();

 private:
  // Name of the timer.
  const std::string name_;
  // Cumulative run time.
  double total_run_time_;
  // Number of times the function was invoked.
  uint64_t total_invocations_;
};

// Timer to check if a function is executed at the specified rate. To use this
// timer, declare an instance of RateChecker at a higher scope than the
// function, and instantiate an Invocation for each function.
// Example:
// ==============================
// // Foo should execute at 10 Hz.
// RateChecker foo_rate_checker_("Foo", 10);
// void Foo() {
//   RateChecker::Invocation invoke(&foo_rate_checker_);
//   // ... Do some stuff ...
// }
// ==============================
class RateChecker {
 public:
  class Invocation {
   public:
    // Primary constructor, starts the timer.
    explicit Invocation(RateChecker* rate_checker);

    // Default destructor, Stops the timer.
    ~Invocation();
   private:
    // Disable copy constructor.
    Invocation(const Invocation&);
    // Disable default constructor.
    Invocation();

   private:
    // Start time.
    const double t_start_;
    // Pointer to rate checker.
    RateChecker* const rate_checker_;
  };

 public:
  // Primary constructor, pass in the function name, or whatever label you wish
  // to identify the timer by.
  explicit RateChecker(const char* name);

  // Default destructor. Print statistics of all invocations.
  ~RateChecker();

 private:
  // Disable copy constructor.
  RateChecker(const RateChecker&);
  // Disable default constructor.
  RateChecker();

 private:
  // Name of the timer.
  const std::string name_;
  // Start time of last iteration.
  double t_last_run_;
  // Total intervals between invocations.
  double total_intervals_;
  // Total execution time.
  double total_run_time_;
  // Number of times the function was invoked.
  uint64_t total_invocations_;
};
#endif  // SRC_UTIL_TIMER_H_
