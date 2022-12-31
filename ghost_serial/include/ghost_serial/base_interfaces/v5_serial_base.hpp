#ifndef GHOST_SERIAL__V5_SERIAL_BASE_HPP
#define GHOST_SERIAL__V5_SERIAL_BASE_HPP

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include <iomanip>
#include <iostream>

#include "ghost_serial/msg_parser/msg_parser.hpp"
#include "ghost_serial/base_interfaces/generic_serial_base.hpp"

namespace ghost_serial
{

    class V5SerialBase : public GenericSerialBase
    {
    public:
        V5SerialBase(
            std::string msg_start_seq,
            int msg_len,
            bool use_checksum = false);

        ~V5SerialBase();

        /**
         * @brief Thread-safe method to read serial port for new msgs. Will read and process a single msg before returning
         * and does not block for a significant period of time. Intended to be called within an external loop.
         *
         * Internally applies COBS decoding and checksum for msg validation. Searches for specified start sequence
         * and then reads for msg_len until null delimiter is found.
         *
         * @param msg_buffer buffer of length msg_len to store incoming serial msgs
         * @return bool if msg was found in serial stream
         */
        bool readMsgFromSerial(unsigned char msg_buffer[]) override;

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
         * @brief Sets any necessary serial port configuration at initialization time
         * 
         * @return bool if port config was set without error
         */
        bool setSerialPortConfig() override;
    };

} // namespace ghost_serial

#endif // GHOST_SERIAL__V5_SERIAL_BASE_HPP