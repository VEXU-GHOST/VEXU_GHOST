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

#ifndef GHOST_SERIAL__V5_SERIAL_BASE_HPP
#define GHOST_SERIAL__V5_SERIAL_BASE_HPP

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>

#include "ghost_serial/base_interfaces/generic_serial_base.hpp"
#include "ghost_serial/msg_parser/msg_parser.hpp"

namespace ghost_serial
{

class V5SerialBase : public GenericSerialBase
{
public:
  V5SerialBase(
    std::string read_msg_start_seq,
    int read_msg_max_len_,
    bool use_checksum = false);

  ~V5SerialBase();

  /**
   * @brief Thread-safe method to read serial port for new msgs. Will read and process a single msg before returning
   * and does not block for a significant period of time. Intended to be called within an external loop.
   *
   * Internally applies COBS decoding and checksum for msg validation. Searches for specified start sequence
   * and then reads for msg_len until null delimiter is found.
   *
   * @param msg_buffer buffer of length msg_len to store incoming serial msgs
   * @return bool if msg was found in serial stream
   */
  bool readMsgFromSerial(std::vector<unsigned char> & msg_buffer, int & parsed_msg_len) override;

private:
  /**
   * @brief Flushes buffer from serial input stream (reader), clearing old data.

   * @return false if flush fails
   */
  bool flushStream() const override;

  /**
   * @brief Returns bytes available to read from serial port file descriptor
   *
   * @return int num_bytes_available
   */
  int getNumBytesAvailable() const override;

  /**
   * @brief Sets any necessary serial port configuration at initialization time
   *
   * @return bool if port config was set without error
   */
  bool setSerialPortConfig() override;
};

} // namespace ghost_serial

#endif // GHOST_SERIAL__V5_SERIAL_BASE_HPP
