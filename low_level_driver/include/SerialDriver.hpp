#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <cstdint>

class SerialDriver
{
public:
    SerialDriver(const std::string &port = "/dev/ttyACM0", int baudrate = 9600);
    ~SerialDriver();

    struct SerialError
    {
        enum Code : int32_t
        {
            NONE = 0,
            PORT_NOT_OPEN = -1,
            UNKNOWN_ERROR = -99
        };
        Code code;
        std::string message;
    };

    SerialError writeLine(const std::string &line);
    SerialError setBaudrate(uint32_t baudrate);
    uint32_t getBaudrate() const { return baudrate_; }

private:
    boost::asio::io_context io_;      // use io_context instead of io_service
    boost::asio::serial_port serial_; // must be initialized with io_context

    std::string port_name_;
    uint32_t baudrate_;
};
