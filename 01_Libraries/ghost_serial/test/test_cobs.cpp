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

#include "ghost_serial/cobs/cobs.hpp"

#include "gtest/gtest.h"

class TestCOBS : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }
};

TEST_F(TestCOBS, testCOBSEncode) {
  unsigned char input_buffer[] = {'t', 'e', '\x00', 's', 't'};
  unsigned char output_buffer[6] = {0, };

  COBS::cobsEncode(input_buffer, sizeof(input_buffer), output_buffer);

  unsigned char expected[] = {3, 't', 'e', 3, 's', 't'};


  for (int i = 0; i < sizeof(expected); i++) {
    ASSERT_EQ(expected[i], output_buffer[i]);
  }
}

TEST_F(TestCOBS, testCOBSDecode) {
  unsigned char input_buffer[] = {3, 't', 'e', 3, 's', 't'};
  unsigned char output_buffer[6] = {0, };

  COBS::cobsDecode(input_buffer, sizeof(input_buffer), output_buffer);

  unsigned char expected[] = {'t', 'e', '\x00', 's', 't'};


  for (int i = 0; i < sizeof(expected); i++) {
    ASSERT_EQ(expected[i], output_buffer[i]);
  }
}

TEST_F(TestCOBS, testCOBSDecodeSerialData) {
  unsigned char input_buffer[] = {0x05, 0x73, 0x6f, 0x75, 0x74, 2, 2, 1, 1, 0};
  unsigned char output_buffer[10] = {0, };
  int num = COBS::cobsDecode(input_buffer, sizeof(input_buffer), output_buffer);
  ASSERT_EQ(num, 9);
  unsigned char expected[] = {'s', 'o', 'u', 't', 0x00, 0x02, 0x00, 0x00};


  for (int i = 0; i < sizeof(expected); i++) {
    ASSERT_EQ(expected[i], output_buffer[i]);
  }
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
