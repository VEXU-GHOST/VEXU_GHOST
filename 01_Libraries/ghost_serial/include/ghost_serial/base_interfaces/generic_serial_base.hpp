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

#ifndef GHOST_SERIAL__GENERIC_SERIAL_BASE_HPP
#define GHOST_SERIAL__GENERIC_SERIAL_BASE_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "../msg_parser/msg_parser.hpp"

#if GHOST_DEVICE == GHOST_JETSON
        #define CROSSPLATFORM_MUTEX_T std::mutex
#elif GHOST_DEVICE == GHOST_V5_BRAIN
        #include "api.h"
        #include "pros/apix.h"
        #define CROSSPLATFORM_MUTEX_T pros::Mutex
#else
        #error "Ghost Device compile flag is not set to valid value"
#endif

namespace ghost_serial
{

class GenericSerialBase
{
public:
  GenericSerialBase(
    std::string write_msg_start_seq,
    std::string read_msg_start_seq,
    int read_msg_max_len,
    bool use_checksum = false);

  ~GenericSerialBase();

  /**
   * @brief Thread-safe method to write msg buffer to serial port. Shares mutex with reader thread,
   * so may block for duration of a serial read (very short period).
   *
   * Applies COBS Encoding to msg before writing to serial.
   * Calculates and appends msg checksum if configured.
   *
   * @param buffer    msg to write to serial
   * @param num_bytes length of msg in bytes
   * @return bool if write was successful
   */
  bool writeMsgToSerial(const unsigned char buffer[], const int num_bytes);

  // Platform specific depending on Serial IO interfaces
  virtual bool readMsgFromSerial(std::vector<unsigned char> & msg_buffer, int & parsed_msg_len) = 0;

  /**
   * @brief Ensures the buffer passed to readMsgFromSerial is of proper size.
   *
   * Throws if msg_buffer is improperly initialized or is not atleast of size read_msg_len_.
   *
   * @param msg_buffer
   */
  void checkReadMsgBufferLength(std::vector<unsigned char> & msg_buffer) const;

protected:
  // Platform specific depending on Serial IO interfaces
  virtual bool flushStream() const = 0;
  virtual int getNumBytesAvailable() const = 0;
  virtual bool setSerialPortConfig() = 0;

  /**
   * @brief Sums all bytes in msg as 8-bit integers to generate checksum for msg validation.
   *
   * @param buffer    msg buffer
   * @param num_bytes length of msg in bytes
   * @return uint8_t checksum
   */
  static uint8_t calculateChecksum(const unsigned char buffer[], const int & num_bytes);

  // Msg Config
  std::string write_msg_start_seq;
  std::string read_msg_start_seq;
  bool use_checksum_;
  int read_msg_max_len_;

  // Serial Status
  CROSSPLATFORM_MUTEX_T serial_io_mutex_;
  std::atomic_bool port_open_;

  // Serial IO File Descriptors
  int serial_write_fd_;
  int serial_read_fd_;

  // Msg Buffers
  std::vector<unsigned char> read_buffer_;
  std::unique_ptr<MsgParser> msg_parser_;
};

} // namespace ghost_serial

#endif // GHOST_SERIAL__GENERIC_SERIAL_BASE_HPP
