#include "serial_interface/serial_interface.hpp"

using namespace std::chrono_literals;

namespace serial_interface{
    /**
     * @brief Construct a new SerialInterface object
     * 
     * @param config_file 
     * @param read_callback_func 
     */
    SerialInterface::SerialInterface(std::string config_file, std::function<void(unsigned char *)> read_callback_func)
    {
        loadConfiguration(config_file);
        read_callback_func_ = read_callback_func;

        // output_string_test_ = 
		// 	"The very thought of you and Am I Blue? A Love Supreme seems far removed."
		// 	"I Get Along Without You, very well, some other nights.";
        output_string_test_ = "Test 1234";
    }
    /**
     * @brief Destroy SerialInterface object
     * 
     */
    SerialInterface::~SerialInterface()
    {
        close(serial_port_fd_);
    }

    /**
     * @brief initializes SerialInterface configuration from YAML file
     * 
     * @param filename 
     */
    void SerialInterface::loadConfiguration(std::string filename)
    {
        config_yaml_ = YAML::LoadFile(filename);
        port_name_ = config_yaml_["port_name"].as<std::string>();
        msg_start_seq_ = config_yaml_["msg_start_seq"].as<std::string>();
        max_msg_len_ = config_yaml_["max_msg_len"].as<int>();
        max_bytes_to_read_ = config_yaml_["max_bytes_to_read"].as<int>();
        max_msg_time_duration_ = std::chrono::duration<int, std::micro>((int)config_yaml_["max_msg_time_duration"].as<int>());
    }

    /**
     * @brief initializes pipe for use as writer event signal
     * 
     */
    void SerialInterface::writePipeInit()
    {
        int write_pipe_fds[2];
        pipe(write_pipe_fds);

        // Store input and output file descriptors of writer pipe
        write_pipe_input_fd_ = write_pipe_fds[1];
        write_pipe_output_fd_ = write_pipe_fds[0];

        // Initialize polling structs
        pollfd_read_write_[0].fd = write_pipe_output_fd_;
        pollfd_read_write_[0].events = POLLIN;
        pollfd_read_write_[1].fd = serial_port_fd_;
        pollfd_read_write_[1].events = POLLIN;
    }

    /**
     * @brief Checks bytes available to read on a given file descriptor
     * 
     * @param fd file descriptor
     * @return int bytes_available
     */
    int SerialInterface::getAvailableBytesRead(int fd){
        int bytes_available;
        ioctl(fd, FIONREAD, &bytes_available);
        return bytes_available;
    }

    /**
     * @brief Shorthand to flush bytes from a file descriptor
     * 
     * @param fd file descriptor
     */
    void SerialInterface::flushStream(int fd){
        ioctl(fd, TCFLSH, 0); // flush receive
    }

    /**
     * @brief Blocks until Serial port is available to open and then initializes.
     * Flushes any old data from serial port. Retries at 100ms intervals after failure to open port.
     */
    void SerialInterface::waitForSerialReady()
    {
        bool is_serial_connected = false;
        while (globals::run && !is_serial_connected)
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
                    int bytes_available = getAvailableBytesRead(serial_port_fd_);
                    std::cout << bytes_available << " Bytes in serial buffer" << std::endl;

                    std::cout << "Flushing Serial Device" << std::endl;
                    flushStream(serial_port_fd_);

                    // Initialize writer pipe
                    writePipeInit();

                    // Success! Break loop
                    is_serial_connected = true;
                }
            }
            catch (const std::exception &e)
            {
                std::cout << "[V5_Serial_Main] Error: " << std::endl;
                std::cout << e.what() << std::endl;
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    bool SerialInterface::parseRawStream(const unsigned char *input_buffer, const int num_bytes, char *packet_decoded)
    {
        int start_seq_count = 0;
        int msg_packet_index = 1;

        uint8_t msg_packet_encoded[max_msg_len_] = {
            0,
        };
        bool msg_found = false;
        // Iterate through serial data
        for (int i = 0; i < num_bytes; i++)
        {
            if (start_seq_count == msg_start_seq_.length())
            {
                // Collect packet from msg buffer
                if (msg_packet_index < max_msg_len_)
                {
                    // Accumulate
                    msg_packet_encoded[msg_packet_index] = input_buffer[i];
                    msg_packet_index++;
                }
                else
                {
                    if (input_buffer[i] == 0x00)
                    {
                        // Process Msg
                        COBS::cobsDecode(msg_packet_encoded, sizeof(msg_packet_encoded), packet_decoded);
                        // int msg_stamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        //                        reader_msg_start_stamp_ - globals::program_start_time)
                        //                        .count();
                        // std::cout << "New Message at: " << msg_stamp_ms << "ms" << std::endl;

                        // Validate checksum
                        // msg_found = true
                    }
                    else
                    {
                        std::cout << "ERROR: Msg exceeds maximum length." << std::endl;
                        std::cout << "Msg Fragment" << std::endl;
                        for (int k = 0; k < num_bytes; k++)
                        {
                            std::cout << std::hex << (int)msg_packet_encoded[k] << " ";
                        }
                        std::cout << std::dec << std::endl;
                    }

                    // Reset state
                    start_seq_count = 0;
                    msg_packet_index = 1;
                }
            }
            else if (start_seq_count >= 0 && start_seq_count < msg_start_seq_.length())
            {
                // Searching for start sequence
                if (input_buffer[i] == msg_start_seq_[start_seq_count])
                {
                    // Input looks like start sequence component
                    start_seq_count++;
                }
                else
                {
                    // Remove PROS Header from msg
                    msg_packet_encoded[0] = input_buffer[i] - msg_start_seq_.length();

                    // No start sequence, reset
                    start_seq_count = 0;
                    msg_packet_index = 1;
                }
            }
            else
            {
                // ERROR
                std::cout << "ERROR in V5 Serial Consumer. start_seq_count = " << start_seq_count << std::endl;
                start_seq_count = 0;
                msg_packet_index = 1; // ?
            }
        }
        return msg_found;
    }

    void SerialInterface::processSerialWrite()
    {
        // Clear Pipe
        // Needs atomic int counter pls
        unsigned char write_byte[2];
        read(write_pipe_output_fd_, &write_byte, sizeof(write_byte));

        std::cout << "Write Requested" << std::endl;
    }

    void SerialInterface::processSerialRead()
    {
    // Read all available bytes from serial input
    int bytes_available = getAvailableBytesRead(serial_port_fd_);
    unsigned char read_buffer[bytes_available] = {0,};
    read(serial_port_fd_, read_buffer, bytes_available);
    write(serial_port_fd_, output_string_test_.c_str(), output_string_test_.length());
    std::cout << read_buffer << std::endl;


    // // Add timestamp and msg to incoming data queue
    // auto msg_stamp = std::chrono::system_clock::now();

    // Send signal to consumer thread



        // unsigned char raw_msg_buffer[max_bytes_to_read_] = {0,};
        // char msg_packet[max_msg_len_] = {0,};

        // // Read buffer length from serial port
        // int num_bytes = read(serial_port_fd_, raw_msg_buffer, sizeof(raw_msg_buffer));

        // if(num_bytes !=0){
        //     // Parse msg from raw buffer
        //     if(parseRawStream(raw_msg_buffer, num_bytes, msg_packet)){
        //         // processMsg(msg_packet);
        //     }
        // }
    }

    /**
     * @brief Processes incoming serial read/write events. Blocks current thread until run_flag is set to false.
     * Efficiently processes Serial IO with user defined messages and read callback function.
     * 
     * @param run_flag atomic boolean reference which signals end of serial loop
     */
    void SerialInterface::runSerialLoop(std::atomic_bool &run_flag)
    {
        while (run_flag)
        {
            try
            {
                auto start = std::chrono::system_clock::now();
                write(serial_port_fd_, output_string_test_.c_str(), output_string_test_.length());

                // Block waiting for read, write, or timeout
                int ret = poll(pollfd_read_write_, 2, 10);

                // One or more file descriptors recieved signal
                if (ret > 0)
                {
                    // Write Event
                    if (pollfd_read_write_[0].revents & POLLIN)
                    {
                        processSerialWrite();
                    }

                    // Read Event
                    if (pollfd_read_write_[1].revents & POLLIN)
                    {
                        // processSerialRead();
                        unsigned char input[131] = {0,};
                        int num_bytes = read(serial_port_fd_, input, sizeof(input));
                        std::cout << input << std::endl;
                        auto end = std::chrono::system_clock::now();
                        std::cout << "Round trip: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << ", " << num_bytes << std::endl;
                        
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
    }

} // namespace serial_interface
