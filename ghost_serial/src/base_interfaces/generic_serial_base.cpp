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
        bool use_checksum): msg_len_(msg_len),
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
                int raw_msg_len = num_bytes + use_checksum_;
                int encoded_msg_len = raw_msg_len + 2;

                // New msg buffer (checksum will add one byte past original msg buffer length)
                unsigned char msg_buffer_raw[raw_msg_len] = {
                    0,
                };

                // Calculate and append checksum byte (if used)
                if (use_checksum_)
                {
                    memcpy(msg_buffer_raw, buffer, num_bytes);
                    msg_buffer_raw[num_bytes] = calculateChecksum(buffer, num_bytes);
                }
                else
                {
                    memcpy(msg_buffer_raw, buffer, raw_msg_len);
                }

                // COBS Encode (Adds leading byte and null delimiter byte)
                unsigned char msg_buffer_encoded[encoded_msg_len] = {
                    0,
                };

                // On V5 Brain, serial output is COBS encoded by default
                #if GHOST_DEVICE != GHOST_V5_BRAIN
                    COBS::cobsEncode(msg_buffer_raw, raw_msg_len, msg_buffer_encoded);
                #endif
                
                // Write to serial port
                std::unique_lock<CROSSPLATFORM_MUTEX_T> write_lock(serial_io_mutex_);
                int ret = write(serial_write_fd_, msg_buffer_encoded, encoded_msg_len);
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
