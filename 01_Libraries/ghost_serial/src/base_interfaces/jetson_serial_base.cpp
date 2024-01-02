#include "ghost_serial/base_interfaces/jetson_serial_base.hpp"

#include <cstring>
#include <exception>

using namespace std::chrono_literals;

namespace ghost_serial {

/**
 * @brief Construct a new JetsonSerialBase object
 *
 * @param config_file
 */
JetsonSerialBase::JetsonSerialBase(std::string write_msg_start_seq,
                                   std::string read_msg_start_seq,
                                   int read_msg_max_len,
                                   bool use_checksum,
                                   bool verbose) :
	GenericSerialBase(
		write_msg_start_seq,
		read_msg_start_seq,
		read_msg_max_len,
		use_checksum),
	verbose_{verbose}{
}

JetsonSerialBase::~JetsonSerialBase(){
	close(serial_read_fd_);
	close(serial_write_fd_);
	port_open_ = false;
}

/**
 * @brief Shorthand to flush bytes from serial port
 */
bool JetsonSerialBase::flushStream() const {
	int ret = ioctl(serial_read_fd_, TCFLSH, 0);
	return (ret != -1) ? true : false;
}

/**
 * @brief Checks bytes available to read from serial port
 *
 * @return int bytes_available
 */
int JetsonSerialBase::getNumBytesAvailable() const {
	int bytes_available;
	ioctl(serial_read_fd_, FIONREAD, &bytes_available);
	return bytes_available;
}

bool JetsonSerialBase::setSerialPortConfig(){
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
	return tcsetattr(serial_read_fd_, TCSANOW, &tty) == 0;
}

bool JetsonSerialBase::trySerialInit(std::string port_name){
	// Attempt to open serial port for read/write
	serial_read_fd_ = open(port_name.c_str(), O_RDWR);
	serial_write_fd_ = serial_read_fd_;

	if(!setSerialPortConfig()){
		std::string err_string = "Error " + std::to_string(errno) + " on port:" + port_name + ", " + strerror(errno);
		throw std::runtime_error(err_string);
	}

	if(serial_read_fd_ < 0){
		// Failed to open, output error msg
		std::string err_string = "Failed to open serial device on " + port_name;
		throw std::runtime_error(err_string);
	}
	else{
		// Serial port now open, flush buffer
		flushStream();

		// Initialize polling structs
		pollfd_read_.fd = serial_read_fd_;
		pollfd_read_.events = POLLIN;

		// Success! Break loop
		port_open_ = true;
		port_name_ = port_name;
		return true;
	}
}

void JetsonSerialBase::printReadBufferDebugInfo(){
	std::cout << "Read Buffer Size: " << read_buffer_.size() << std::endl;
	for(const auto& byte : read_buffer_){
		std::cout << "x" << std::setw(2) << std::setfill('0') << std::hex << (int) byte;
	}
	std::cout << std::dec << std::endl;
}

bool JetsonSerialBase::readMsgFromSerial(std::vector<unsigned char> &msg_buffer, int & parsed_msg_len){
	if(port_open_){
		// Block waiting for read or timeout (1s)
		int ret = poll(&pollfd_read_, 1, 1000);

		// File descriptor recieves signal
		if((pollfd_read_.revents & POLLIN) == POLLIN){
			// Lock serial port mutex from writes and read serial data
			std::unique_lock<CROSSPLATFORM_MUTEX_T> read_lock(serial_io_mutex_);

			// Read available bytes, up to size of raw_serial_buffer (two msgs - one byte)
			int bytes_to_read = std::min(getNumBytesAvailable(), (int)read_buffer_.size());
			int num_bytes_read = read(serial_read_fd_, read_buffer_.data(), bytes_to_read);
			read_lock.unlock();

			// Extract any msgs from serial stream and return if msg is found
			if(num_bytes_read > 0){
				if(verbose_){
					std::cout << "Read " << num_bytes_read << " bytes" << std::endl;
					printReadBufferDebugInfo();
				}

				checkReadMsgBufferLength(msg_buffer); // Throws if msg_buffer is misconfigured
				return msg_parser_->parseByteStream(read_buffer_.data(), num_bytes_read, msg_buffer.data(), parsed_msg_len);
			}
			else if(num_bytes_read == -1){
				perror("Error");
			}
		}
		else if(ret == -1){
			// Poll Error
			throw std::system_error(errno, std::generic_category());
		}
	}
	return false;
}

} // namespace serial_interface