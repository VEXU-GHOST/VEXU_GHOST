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
        std::string msg_start_seq,
        int msg_len,
        bool use_checksum): GenericSerialBase(
                                msg_start_seq,
                                msg_len,
                                use_checksum)
    {
        serial_read_fd_ = fileno(stdin);
        serial_write_fd_ = fileno(stdout);
		port_open_ = true;
    }

    V5SerialBase::~V5SerialBase(){
        
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
        pros::c::fdctl(serial_read_fd_, DEVCTL_FIONREAD, NULL);
        return bytes_available;
    }

    bool V5SerialBase::setSerialPortConfig()
    {
        return true;
    }

    bool V5SerialBase::readMsgFromSerial(unsigned char msg_buffer[], int & parsed_msg_len)
    {
        if (port_open_)
        {
            try
            {
                // Lock serial port mutex from writes and read serial data
                std::unique_lock<pros::Mutex> read_lock(serial_io_mutex_);

                // Read available bytes, up to size of raw_serial_buffer (two msgs - one byte)
                int bytes_to_read = std::min(getNumBytesAvailable(), (int)read_buffer_.size());
                int num_bytes_read = read(serial_read_fd_, read_buffer_.data(), bytes_to_read);
                read_lock.unlock();

                // Extract any msgs from serial stream and return if msg is found
                if (num_bytes_read > 0)
                {
                    return msg_parser_->parseByteStream(read_buffer_.data(), num_bytes_read, msg_buffer, parsed_msg_len);
                }
                else if (num_bytes_read == -1)
                {
                    // TODO: stdout is a no-go on this device. We need to add log files and SD card.
                    // perror("Error");
                }
            }
            catch (const std::exception &e)
            {
                // TODO: See logging note above.
                // std::cout << "[Serial Reader Error]: " << e.what() << std::endl;
            }
        }
        return false;
    }

} // namespace serial_interface