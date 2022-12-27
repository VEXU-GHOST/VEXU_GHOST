#include "serial_interface/serial_interface.hpp"

#include <cstring>
#include <exception>

using namespace std::chrono_literals;

namespace ghost_serial
{
    /**
     * @brief Construct a new SerialInterface object
     *
     * @param config_file
     */
    SerialInterface::SerialInterface(
        std::string port_name,
        std::string msg_start_seq,
        int msg_len,
        bool use_checksum,
        bool verbose) : msg_len_(msg_len),
                        port_name_(port_name),
                        msg_start_seq_(msg_start_seq),
                        use_checksum_(use_checksum),
                        verbose_(verbose),
                        port_open_(false)
    {
        // Reads a maximum of (two msgs - one byte) at once
        raw_serial_buffer_ = std::vector<unsigned char>(2 * (msg_len + use_checksum_ + 2) - 1);
        msg_parser_ = std::make_unique<MsgParser>(msg_len, msg_start_seq_, use_checksum_);
    }

    /**
     * @brief Destroy SerialInterface object
     *
     */
    SerialInterface::~SerialInterface()
    {
        // Should disable read/write interfaces
        port_open_ = false;

        // Acquire lock (in case any read/writes are in progress)
        std::unique_lock<std::mutex> write_lock(serial_io_mutex_);

        // Cleanup
        close(serial_port_fd_);
    }

    /**
     * @brief Shorthand to flush bytes from serial port
     */
    void SerialInterface::flushStream() const
    {
        ioctl(serial_port_fd_, TCFLSH, 0);
    }

    /**
     * @brief Checks bytes available to read from serial port
     *
     * @return int bytes_available
     */
    int SerialInterface::getNumBytesAvailable() const
    {
        int bytes_available;
        ioctl(serial_port_fd_, FIONREAD, &bytes_available);
        return bytes_available;
    }

    uint8_t SerialInterface::calculateChecksum(const unsigned char buffer[], const int &num_bytes) const
    {
        uint8_t checksum = 0;
        for (int i = 0; i < num_bytes; i++)
        {
            checksum += buffer[i];
        }
        return checksum;
    }

    bool SerialInterface::setSerialPortConfig()
    {
        // Serial Port Configuration
        struct termios tty;

        tty.c_cflag &= ~PARENB;        // Clear parity bit
        tty.c_cflag &= ~CSTOPB;        // Single stop bit
        tty.c_cflag &= ~CSIZE;         // Clear all the size bits
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON; // Disable Canonical Mode (removes waiting for EOL and disables backspaces)

        // Echo bits
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

        // Input Modes
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        // Output Modes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        // VMIN and VTIME
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        // Set Baud Rate
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Save tty settings, also checking for error
        return tcsetattr(serial_port_fd_, TCSANOW, &tty) != 0;
    }

    /**
     * @brief Blocks until Serial port is available to open and then initializes.
     * Flushes any old data from serial port. Retries at 100ms intervals after failure to open port.
     */
    bool SerialInterface::trySerialOpen()
    {
        try
        {
            // Attempt to open serial port for read/write
            serial_port_fd_ = open(port_name_.c_str(), O_RDWR);

            if (setSerialPortConfig())
            {
                std::string err_string = "Error " + std::to_string(errno) + ", " + strerror(errno);
                throw std::runtime_error(err_string);
            }

            if (serial_port_fd_ < 0)
            {
                // Failed to open, output error msg
                std::string err_string = "Failed to open serial device on " + port_name_;
                throw std::runtime_error(err_string);
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
            if (verbose_)
            {
                std::cout << "[Serial Interface] Error: " << e.what() << std::endl;
            }
        }
        return false;
    }

    bool SerialInterface::writeMsgToSerial(const unsigned char buffer[], const int num_bytes)
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
                COBS::cobsEncode(msg_buffer_raw, raw_msg_len, msg_buffer_encoded);

                // Write to serial port
                std::unique_lock<std::mutex> write_lock(serial_io_mutex_);
                int ret = write(serial_port_fd_, msg_buffer_encoded, encoded_msg_len);
                write_lock.unlock();

                if (ret != -1)
                {
                    succeeded = true;
                }
            }
            catch (std::exception &e)
            {
                std::cout << "Error writing to serial: " << e.what() << std::endl;
            }
        }
        return succeeded;
    }

    bool SerialInterface::readMsgFromSerial(unsigned char msg_buffer[])
    {
        if (port_open_)
        {
            try
            {
                // Block waiting for read or timeout (1s)
                int ret = poll(&pollfd_read_, 1, 1000);

                // File descriptor recieves signal
                if ((pollfd_read_.revents & POLLIN) == POLLIN)
                {
                    // Lock serial port mutex from writes and read serial data
                    std::unique_lock<std::mutex> read_lock(serial_io_mutex_);

                    // Read available bytes, up to size of raw_serial_buffer (two msgs - one byte)
                    int bytes_to_read = std::min(getNumBytesAvailable(), (int)raw_serial_buffer_.size());
                    int num_bytes_read = read(serial_port_fd_, raw_serial_buffer_.data(), bytes_to_read);
                    read_lock.unlock();

                    // Extract any msgs from serial stream and return if msg is found
                    if (num_bytes_read > 0)
                    {
                        return msg_parser_->parseByteStream(raw_serial_buffer_.data(), num_bytes_read, msg_buffer);
                    }
                    else if (num_bytes_read == -1)
                    {
                        perror("Error");
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
