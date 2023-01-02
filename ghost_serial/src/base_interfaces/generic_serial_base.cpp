#include "ghost_serial/base_interfaces/generic_serial_base.hpp"

#include <cstring>
#include <exception>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ghost_serial
{
    /**
     * @brief Construct a new GenericSerialBase object
     *
     * @param config_file
     */
    GenericSerialBase::GenericSerialBase(
        std::string msg_start_seq,
        int msg_len,
        bool use_checksum): max_msg_len_(msg_len),
                            msg_start_seq_(msg_start_seq),
                            use_checksum_(use_checksum),
                            port_open_(false)
    {
        // Reads a maximum of (two msgs - one byte) at once
        read_buffer_ = std::vector<unsigned char>(2 * (msg_len + use_checksum_ + 2) - 1);
        msg_parser_ = std::make_unique<MsgParser>(msg_len, msg_start_seq_, use_checksum_);
    }

    /**
     * @brief Destroy GenericSerialBase object
     *
     */
    GenericSerialBase::~GenericSerialBase()
    {
        // Acquire lock (in case any read/writes are in progress)
        std::unique_lock<CROSSPLATFORM_MUTEX_T> close_lock(serial_io_mutex_);

        // For Jetson, we use the same port for read and write
        if(serial_write_fd_ == serial_read_fd_){
            close(serial_read_fd_);
        }
        else{
            close(serial_write_fd_);
            close(serial_read_fd_);
        }
    }

    uint8_t GenericSerialBase::calculateChecksum(const unsigned char buffer[], const int &num_bytes) const
    {
        uint8_t checksum = 0;
        for (int i = 0; i < num_bytes; i++)
        {
            checksum += buffer[i];
        }
        return checksum;
    }

    bool GenericSerialBase::writeMsgToSerial(const unsigned char buffer[], const int num_bytes)
    {
        bool succeeded = false;
        if (port_open_)
        {
            try
            {
                // Raw msg buffer (checksum will add one byte past original msg buffer length)
                int raw_msg_len = num_bytes + use_checksum_;
                unsigned char raw_msg_buffer[raw_msg_len] = {
                    0,
                };

                // Calculate and append checksum byte (if used)
                if (use_checksum_)
                {
                    memcpy(raw_msg_buffer, buffer, num_bytes);
                    raw_msg_buffer[num_bytes] = calculateChecksum(buffer, num_bytes);
                }
                else
                {
                    memcpy(raw_msg_buffer, buffer, raw_msg_len);
                }

                // On V5 Brain, serial output is COBS encoded by default, otherwise, encode it (adding two bytes)
                #if GHOST_DEVICE != GHOST_V5_BRAIN
                    // COBS Encode (Adds leading byte and null delimiter byte)
                    int write_buffer_len = raw_msg_len + 2;
                    unsigned char write_buffer[write_buffer_len] = {
                        0,
                    };
                    COBS::cobsEncode(raw_msg_buffer, raw_msg_len, write_buffer);
                #else
                    int write_buffer_len = raw_msg_len;
                    unsigned char * write_buffer = raw_msg_buffer;
                #endif
                
                // Write to serial port
                std::unique_lock<CROSSPLATFORM_MUTEX_T> write_lock(serial_io_mutex_);
                int ret = write(serial_write_fd_, write_buffer, write_buffer_len);
                write_lock.unlock();

                if (ret != -1)
                {
                    succeeded = true;
                }
            }
            catch (std::exception &e)
            {
                #ifdef GHOST_DEBUG_VERBOSE
                    #if GHOST_DEVICE == GHOST_JETSON
                        std::cout << "Error writing to serial: " << e.what() << std::endl;
                    #endif
                #endif
            }
        }
        return succeeded;
    }
} // namespace serial_interface
