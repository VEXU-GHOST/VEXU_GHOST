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

#include "ghost_serial/base_interfaces/v5_serial_base.hpp"

#include <cstring>
#include <exception>

#include "pros/apix.h"

using namespace std::chrono_literals;

namespace ghost_serial
{

/**
 * @brief Construct a new V5SerialBase object
 *
 * @param config_file
 */
V5SerialBase::V5SerialBase(
  std::string read_msg_start_seq,
  int read_msg_max_len,
  bool use_checksum)
: GenericSerialBase(
    "sout",
    read_msg_start_seq,
    read_msg_max_len,
    use_checksum)
{
  serial_read_fd_ = fileno(stdin);
  serial_write_fd_ = fileno(stdout);
  port_open_ = true;
}

V5SerialBase::~V5SerialBase()
{
}

/**
 * @brief Shorthand to flush bytes from serial port
 */
bool V5SerialBase::flushStream() const
{
  // Check num bytes available to read
  int bytes_available = getNumBytesAvailable();

  // Allocate garbage buffer to read data, where buffer is deallocated when it goes out of scope
  // Not ideal, but there's no fdctl macros to flush the port
  std::vector<unsigned char> flush_buffer(bytes_available);
  int ret = read(serial_read_fd_, flush_buffer.data(), bytes_available);

  return (ret != -1) ? true : false;
}

/**
 * @brief Checks bytes available to read from serial port
 *
 * @return int bytes_available
 */
int V5SerialBase::getNumBytesAvailable() const
{
  int bytes_available;
  bytes_available = pros::c::fdctl(serial_read_fd_, DEVCTL_FIONREAD, NULL);
  return bytes_available;
}

bool V5SerialBase::setSerialPortConfig()
{
  return true;
}

bool V5SerialBase::readMsgFromSerial(std::vector<unsigned char> & msg_buffer, int & parsed_msg_len)
{
  int max_read_bytes = read_msg_max_len_ + use_checksum_ + read_msg_start_seq.length() + 2;
  if (port_open_) {
    try {
      // Lock serial port mutex from writes and read serial data
      // std::unique_lock<CROSSPLATFORM_MUTEX_T> read_lock(serial_io_mutex_);

      // Block until one full msg is read
      int num_bytes_read = read(serial_read_fd_, read_buffer_.data(), max_read_bytes);

      // read_lock.unlock();

      // Extract any msgs from serial stream and return if msg is found
      if (num_bytes_read > 0) {
        checkReadMsgBufferLength(msg_buffer);                         // Throws if msg_buffer is misconfigured
        return msg_parser_->parseByteStream(
          read_buffer_.data(), num_bytes_read,
          msg_buffer.data(), parsed_msg_len);
      } else if (num_bytes_read == -1) {
        // TODO: stdout is a no-go on this device. We need to add log files and SD card.
        // perror("Error");
      }
    } catch (const std::exception & e) {
      // TODO: See logging note above.
      // std::cout << "[Serial Reader Error]: " << e.what() << std::endl;
    }
  }
  return false;
}

} // namespace serial_interface
