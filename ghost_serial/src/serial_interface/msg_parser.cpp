#include "serial_interface/msg_parser.hpp"

namespace ghost_serial
{
    MsgParser::MsgParser(int msg_len, std::string msg_start_seq, bool use_checksum) : 
        msg_len_(msg_len),
        msg_start_seq_(msg_start_seq),
        msg_packet_index_(1),
        start_seq_index_(0),
        use_checksum_(use_checksum)
    {
        // Allocate buffers to store serial data
        incoming_msg_buffer_ = std::vector<unsigned char>(msg_len_ + 2); // Adds COBS Start Byte and Null Delimiter
        parsed_msg_buffer_ = std::vector<char>(msg_len_);
    }

    bool MsgParser::parseByteStream(unsigned char *raw_data_buffer, int num_bytes, unsigned char *parsed_msg)
    {
        bool msg_found = false;
        // Iterate through serial data
        for (int i = 0; i < num_bytes; i++)
        {
            if (start_seq_index_ == msg_start_seq_.length())
            {
                // Collect packet from msg buffer
                // Add 1 for COBS Encoding Start Byte
                if (msg_packet_index_ < msg_len_ + 1)
                {
                    // Accumulate
                    incoming_msg_buffer_[msg_packet_index_] = raw_data_buffer[i];
                    msg_packet_index_++;
                }
                else{
                    if (raw_data_buffer[i] == 0x00)
                    {
                        // Process Msg
                        COBS::cobsDecode(incoming_msg_buffer_.data(), sizeof(incoming_msg_buffer_), parsed_msg);

                        // Validate checksum
                        if(use_checksum_){
                            uint8_t checksum_byte = parsed_msg[msg_len_-1];
                            for(int b = 0; b < msg_len_ - 1; b++){
                                checksum_byte -= parsed_msg[b];
                            }
                            msg_found = (checksum_byte == 0);
                        }
                        else{
                            msg_found = true;
                        }
                    }
                    start_seq_index_ = 0;
                    msg_packet_index_ = 1;
                }
            }
            else if (start_seq_index_ < msg_start_seq_.length())
            {
                // Searching for start sequence
                if (raw_data_buffer[i] == msg_start_seq_[start_seq_index_])
                {
                    // Input looks like start sequence component
                    start_seq_index_++;
                }
                else
                {
                    // Remove PROS Header from msg
                    incoming_msg_buffer_[0] = raw_data_buffer[i] - msg_start_seq_.length();

                    // No start sequence, reset
                    start_seq_index_ = 0;
                    msg_packet_index_ = 1;
                }
            }
            else
            {
                // ERROR
                start_seq_index_ = 0;
                msg_packet_index_ = 1;
            }
        }
        return msg_found;
    }

} // namespace ghost_serial