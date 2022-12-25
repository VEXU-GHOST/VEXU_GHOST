#include "serial_interface/serial_interface.hpp"

using namespace std::chrono_literals;

namespace ghost_serial{
    /**
     * @brief Construct a new SerialInterface object
     * 
     * @param config_file 
     */
    SerialInterface::SerialInterface(std::string port_name, std::string msg_start_seq, int incoming_msg_len) : port_open_(false)
    {
        port_name_ = port_name;
        incoming_msg_len_ = incoming_msg_len;
        msg_start_seq_ = msg_start_seq;
        raw_serial_buffer_ = std::vector<unsigned char>(2*incoming_msg_len_);
        incoming_msg_ = std::vector<unsigned char>(incoming_msg_len_);

        msg_parser_ = std::make_unique<MsgParser>(incoming_msg_len_, msg_start_seq_);
    }

    /**
     * @brief Destroy SerialInterface object
     * 
     */
    SerialInterface::~SerialInterface()
    {
        port_open_ = false;
        std::unique_lock<std::mutex> write_lock(serial_io_mutex_);
        close(serial_port_fd_);
    }

    /**
     * @brief Shorthand to flush bytes from serial port
     */
    void SerialInterface::flushStream(){
        ioctl(serial_port_fd_, TCFLSH, 0); // flush receive
    }

    /**
     * @brief Checks bytes available to read from serial port
     *
     * @return int bytes_available
     */
    int SerialInterface::getNumBytesAvailable()
    {
        int bytes_available;
        ioctl(serial_port_fd_, FIONREAD, &bytes_available);
        return bytes_available;
    }

    /**
     * @brief Blocks until Serial port is available to open and then initializes.
     * Flushes any old data from serial port. Retries at 100ms intervals after failure to open port.
     */
    bool SerialInterface::trySerialOpen()
    {
        try
        {
            // Attempt to open serial port for read/write, non_blocking
            serial_port_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);

            if (serial_port_fd_ < 0)
            {
                // Failed to open, output error msg
                std::string err_string = "Failed to open serial device on " + port_name_;
                perror(err_string.c_str());
            }
            else
            {
                // Serial port now open, flush buffer
                flushStream();

                // Initialize polling structs
                pollfd_read_.fd = serial_port_fd_;
                pollfd_read_.events = POLLIN;

                // Success! Break loop
                port_open_ = true;
                return true;
            }
        }
        catch (const std::exception &e)
        {
            std::cout << "[V5_Serial_Main] Error: " << std::endl << e.what() << std::endl;
        }
        return false;
    }

    bool SerialInterface::writeMsgToSerial(unsigned char * buffer, int num_bytes){
        bool succeeded  = false;
        if(port_open_){
            try{
                std::unique_lock<std::mutex> write_lock(serial_io_mutex_);
                int ret = write(serial_port_fd_, buffer, num_bytes);
                write_lock.unlock();
                if(ret != -1){
                    succeeded = true;
                } 
            }
            catch(std::exception &e){
                std::cout << "Error writing to serial: " << e.what() << std::endl;
            }
        }
        return succeeded;
    }

    bool SerialInterface::readMsgFromSerial(unsigned char msg_buffer[])
    {   
        if(port_open_){
            try
            {
                // Block waiting for read or timeout
                int ret = poll(&pollfd_read_, 2, 100);

                // File descriptor recieves signal
                if (ret > 0)
                {
                    std::unique_lock<std::mutex> read_lock(serial_io_mutex_);
                    int bytes_available = getNumBytesAvailable();
                    int num_bytes_read = read(serial_port_fd_, raw_serial_buffer_.data(), bytes_available);
                    read_lock.unlock();

                    bool msg_found = msg_parser_->parseByteStream(raw_serial_buffer_.data(), num_bytes_read, msg_buffer);

                    if(msg_found){
                        return true;
                    }
                }
                else if (ret == -1)
                {
                    // Poll Error
                    throw std::system_error(errno, std::generic_category());
                }
            }
            catch (const std::exception &e)
            {
                std::cout << "[Serial Reader Error]: " << e.what() << std::endl;
            }
        }
        return false;
    }

} // namespace serial_interface
