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

#ifndef GHOST_ROS__STREAM_PARSER_HPP
#define GHOST_ROS__STREAM_PARSER_HPP

#include <iostream>
#include <string>
#include <vector>

#include "../cobs/cobs.hpp"

namespace ghost_serial
{

class MsgParser
{
public:
  MsgParser(int msg_len, std::string msg_start_seq, bool use_checksum = false);
  ~MsgParser()
  {
  }

  /**
   * @brief Iterates through stream of bytes, searching for msg start sequence, and then extracts msg.
   * Will not register msgs that do not match MsgParser msg_len. Intended to be called repeatedly with
   * sequential serial data.
   *
   * In general, num_bytes is not intended to exceed 2*msg_len - 1 bytes. If multiple msgs exist in array,
   * function will (probably) return the most recent complete msg (closest to end of array).
   *
   *
   * @param raw_data_buffer   array containing raw serial data
   * @param num_bytes         number of bytes to read in raw_data_buffer
   * @param parsed_msg        array of size msg_len to store extracted msgs
   * @return bool whether msg is found
   */
  bool parseByteStream(
    const unsigned char raw_data_buffer[], const int num_bytes,
    unsigned char parsed_msg[], int & parsed_msg_len);

private:
  // Config params
  std::string msg_start_seq_;
  int max_msg_len_;
  bool use_checksum_;

  // Serial Buffers
  std::vector<unsigned char> incoming_msg_buffer_;
  std::vector<char> parsed_msg_buffer_;

  // Variables for parsing msg stream
  uint8_t start_seq_index_;
  int msg_packet_index_;
};

} // namespace ghost_serial

#endif // GHOST_ROS__STREAM_PARSER_HPP
