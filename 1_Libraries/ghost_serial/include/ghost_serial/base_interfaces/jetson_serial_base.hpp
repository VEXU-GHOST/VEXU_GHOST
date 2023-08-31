#ifndef GHOST_SERIAL__JETSON_SERIAL_BASE_HPP
#define GHOST_SERIAL__JETSON_SERIAL_BASE_HPP

#include <mutex>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>

#include "../msg_parser/msg_parser.hpp"
#include "ghost_serial/base_interfaces/generic_serial_base.hpp"

namespace ghost_serial {

class JetsonSerialBase : public GenericSerialBase {
public:
	JetsonSerialBase(std::string write_msg_start_seq,
	                 std::string read_msg_start_seq,
	                 int read_msg_max_len,
	                 bool use_checksum = false,
	                 bool verbose = false);

	~JetsonSerialBase();

	/**
	 * @brief closes any open ports, effectively resetting state to default
	 *
	 * Internally, this closes file descriptions and may propogate exceptions returned from close()
	 */
	void closeSerialPort();

	/**
	 * @brief Attempts to open serial port for read/write and set port configuration.
	 *
	 * THROWS runtime errors when serial device is not available or fails to open.
	 *
	 * @returns if init is successful or not
	 */
	bool trySerialInit(std::string port_name);

	/**
	 * @brief Thread-safe method to read serial port for new msgs. Blocks until new msg is recieved
	 * or 1s timeout elapses. Implementation uses Poll() system call to avoid busy-waiting. Highly efficient!
	 *
	 * Internally applies COBS decoding and checksum for msg validation. Searches for specified start sequence
	 * and then reads for msg_len until null delimiter is found.
	 *
	 * THROWS system_error if poll returns -1.
	 *
	 * @param msg_buffer buffer of length msg_len to store incoming serial msgs
	 * @return bool if msg was found in serial stream
	 */
	bool readMsgFromSerial(unsigned char msg_buffer[], int & parsed_msg_len) override;

private:
	/**
	 * @brief Flushes buffer from serial input stream (reader), clearing old data.

	 * @return false if flush fails
	 */
	bool flushStream() const override;

	/**
	 * @brief Returns bytes available to read from serial port file descriptor
	 *
	 * @return int num_bytes_available
	 */
	int getNumBytesAvailable() const override;

	/**
	 * @brief Sets termios flags for serial port settings. Follows this nearly verbatim:
	 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
	 *
	 * @return true if config was set without error
	 */
	bool setSerialPortConfig() override;

	// Config params
	std::string port_name_;
	bool verbose_;

	// Poll Config Strictire
	struct pollfd pollfd_read_;
};

} // namespace ghost_serial

#endif // GHOST_SERIAL__JETSON_SERIAL_BASE_HPP