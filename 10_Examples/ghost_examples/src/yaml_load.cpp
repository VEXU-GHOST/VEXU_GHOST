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

#include <iostream>
#include "yaml-cpp/yaml.h"

int main(int argc, char * argv[])
{
  std::string config_path = std::string(getenv("VEXU_HOME")) +
    "/10_Examples/ghost_examples/config/example.yaml";
  YAML::Node config = YAML::LoadFile(config_path);
  const std::string string_val = config["ns1"]["ns2"]["string_val"].as<std::string>();
  const int an_integer = config["ns1"]["ns2"]["an_int"].as<int>();
  const double some_number = config["ns1"]["ns2"]["a_double"].as<double>();

  std::cout << "Example String: " << string_val << std::endl;
  std::cout << "Example Integer: " << an_integer << std::endl;
  std::cout << "Example Double: " << some_number << std::endl;

  std::cout << "Change example.yaml and run again!" << std::endl;

  return 0;
}
