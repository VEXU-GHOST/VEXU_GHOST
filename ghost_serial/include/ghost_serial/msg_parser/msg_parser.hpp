/**
 * All original serial library files reside under the ghost_serial ROS package.
 * Symlinks have been created in the ghost_pros project to compile the files to the V5 Brain from the same source files.
 * Relative paths in Symlinks mean it should work across machines via Github.

 * If I was more knowledgeable with Makefiles, this would be linked as a static library, but alas I am not. :(
*/

#ifndef GHOST_ROS__STREAM_PARSER_HPP
#define GHOST_ROS__STREAM_PARSER_HPP

#include <string>
#include <iostream>
#include <vector>

#include "../cobs/cobs.hpp"

namespace ghost_serial{

class MsgParser {
    public:
        MsgParser(int msg_len, std::string msg_start_seq, bool use_checksum = false);
        ~MsgParser(){}

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
        bool parseByteStream(const unsigned char raw_data_buffer[], const int num_bytes, unsigned char parsed_msg[]);

    private:
        // Config params
        std::string msg_start_seq_;
        int msg_len_;
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