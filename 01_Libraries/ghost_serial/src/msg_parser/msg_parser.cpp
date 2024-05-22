/**
 * All original serial library files reside under the ghost_serial ROS package.
 * Symlinks have been created in the ghost_pros project to compile the files to the V5 Brain from the same source files.
 * Relative paths in Symlinks mean it should work across machines via Github.

 * If I was more knowledgeable with Makefiles, this would be linked as a static library, but alas I am not. :(
 */

#include <cstring>
#include "ghost_serial/msg_parser/msg_parser.hpp"

namespace ghost_serial
{

MsgParser::MsgParser(int max_msg_len, std::string msg_start_seq, bool use_checksum)
: max_msg_len_(max_msg_len),
  msg_start_seq_(msg_start_seq),
  msg_packet_index_(1),
  use_checksum_(use_checksum),
  start_seq_index_(0)
{
  // Allocate buffers to store serial data
  incoming_msg_buffer_ = std::vector<unsigned char>(max_msg_len_ + use_checksum_ + 2);       // Adds COBS Start Byte and Null Delimiter
  parsed_msg_buffer_ = std::vector<char>(max_msg_len_);
}

bool MsgParser::parseByteStream(
  const unsigned char raw_data_buffer[], const int num_bytes,
  unsigned char parsed_msg[], int & parsed_msg_len)
{
  bool msg_found = false;
  uint8_t checksum_byte = 0;

  // Iterate through serial data
  for (int i = 0; i < num_bytes; i++) {
    if (start_seq_index_ == msg_start_seq_.length()) {           // Found start sequence, now reading msg
      // Collect msg from serial buffer
      // Add 2 for COBS Encoding
      if (msg_packet_index_ < max_msg_len_ + use_checksum_ + 2) {
        if (raw_data_buffer[i] == 0x00) {                       // Msg end delimiter
          // Length of variable size msg, add 1 to ndex for length, remove encoding bytes and checksum
          parsed_msg_len = msg_packet_index_ + 1 - 2 - use_checksum_;
          // Apply COBS Decode to raw msg
          // cobsDecode requires size of input and output buffers to be the same.
          unsigned char decoded_msg[parsed_msg_len + use_checksum_ + 2] = {
            0,
          };
          COBS::cobsDecode(
            incoming_msg_buffer_.data(), parsed_msg_len + use_checksum_ + 2, decoded_msg);

          // Thus, copy ONLY the msg from cobsDecode output to parsed_msg
          memcpy(parsed_msg, decoded_msg, parsed_msg_len);

          // Validate checksum
          if (use_checksum_) {
            // Checksum is one byte past end of parsed_msg
            checksum_byte = decoded_msg[max_msg_len_];
            for (int b = 0; b < max_msg_len_; b++) {
              checksum_byte -= parsed_msg[b];
            }
            msg_found = (checksum_byte == 0);
          } else {
            msg_found = true;
          }

          // Reset
          start_seq_index_ = 0;
          msg_packet_index_ = 1;                               // COBS leading byte
        } else {
          // Accumulate
          incoming_msg_buffer_[msg_packet_index_] = raw_data_buffer[i];
          msg_packet_index_++;
        }
      } else {
        // Reset
        start_seq_index_ = 0;
        msg_packet_index_ = 1;                         // COBS leading byte
      }
    } else if (start_seq_index_ < msg_start_seq_.length()) {         // looking for msg start sequence
      // Searching for start sequence
      if (raw_data_buffer[i] == msg_start_seq_[start_seq_index_]) {
        // Input looks like start sequence component
        start_seq_index_++;
      } else {
        // Once we detect start sequence, we have already missed COBS leading byte.
        // Store the last byte we read every loop while we search for msg start
        incoming_msg_buffer_[0] = raw_data_buffer[i] - msg_start_seq_.length();

        // No start sequence, reset
        start_seq_index_ = 0;
        msg_packet_index_ = 1;                         // COBS leading byte
      }
    } else {
      // Error, reset
      start_seq_index_ = 0;
      msg_packet_index_ = 1;                   // COBS leading byte
    }
  }
  return msg_found;
}

} // namespace ghost_serial
