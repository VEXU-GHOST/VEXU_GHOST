
#ifndef GHOST_ROS__STREAM_PARSER_HPP
#define GHOST_ROS__STREAM_PARSER_HPP

#include <string>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../cobs/cobs.hpp"
#include "yaml-cpp/yaml.h"

namespace ghost_serial{

class MsgParser {
    public:
        MsgParser(int msg_len, std::string msg_start_seq, bool use_checksum = false);
        ~MsgParser(){}

        bool parseByteStream(unsigned char * raw_data_buffer, int num_bytes, unsigned char * parsed_msg);

    private:
        // Serial Buffers
        std::vector<unsigned char> incoming_msg_buffer_;
        std::vector<char> parsed_msg_buffer_;

        // Variables for parsing msg stream
        uint8_t start_seq_index_;
        std::string msg_start_seq_;
        int msg_packet_index_;
        int msg_len_;
        bool use_checksum_;
};

} // namespace ghost_serial

#endif // GHOST_ROS__STREAM_PARSER_HPP