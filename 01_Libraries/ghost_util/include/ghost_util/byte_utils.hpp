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

#include <stdexcept>
#include <vector>
#include <stdint.h>

namespace ghost_util
{

extern const uint32_t BITMASK_ARR_32BIT[32];

inline bool isBigEndian()
{
  unsigned int i = 1;
  char * c = (char *)&i;
  if (*c) {
    return false;
  } else {
    return true;
  }
}

inline void setBit(unsigned char & byte, int bit_num, bool val)
{
  if (bit_num >= 8) {
    throw std::runtime_error("[ghost_util::setBit] Error: bit_num must be between 0 and 7.");
  }
  if (val) {
    byte |= BITMASK_ARR_32BIT[bit_num];
  } else {
    byte &= ~BITMASK_ARR_32BIT[bit_num];
  }
}

inline bool getBit(unsigned char byte, int bit_num)
{
  if (bit_num >= 8) {
    throw std::runtime_error("[ghost_util::setBit] Error: bit_num must be between 0 and 7.");
  }
  return byte & BITMASK_ARR_32BIT[bit_num];
}

inline std::vector<bool> unpackByte(unsigned char val)
{
  std::vector<bool> bit_arr;
  bit_arr.resize(8);
  for (int i = 0; i < 8; i++) {
    auto index = (isBigEndian()) ? i : 7 - i;
    bit_arr[i] = val & BITMASK_ARR_32BIT[index];
  }
  return bit_arr;
}

inline unsigned char packByte(const std::vector<bool> & bool_arr)
{
  if (bool_arr.size() != 8) {
    throw std::runtime_error(
            "[ghost_util::packByte] Error: bool array must be of size 8. Size: " + std::to_string(
              bool_arr.size()));
  }
  unsigned char byte = 0;
  for (int i = 0; i < 8; i++) {
    auto index = (isBigEndian()) ? i : 7 - i;
    setBit(byte, index, bool_arr[i]);
  }
  return byte;
}

} // namespace ghost_util
