#include "robot_arm_driver/SerialDriver.hpp"
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <thread>

SerialDriver::SerialDriver(const std::string &_port, int _baudrate) : io(), serial(io, _port), port_name(_port), baudrate(_baudrate)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(this->baudrate));

    // arduino defaults
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::character_size(8));
}

SerialDriver::~SerialDriver()
{
    if (serial.is_open())
    {
        serial.close();
    }
}

SerialDriver::SerialError SerialDriver::writeLine(const std::string &line)
{
    if (!serial.is_open())
    {
        std::cerr << "Serial port is not open." << std::endl;
        return {SerialError::PORT_NOT_OPEN, "Serial port is not open."};
    }

    std::cout << "Sending line: " << line << std::endl;

    boost::asio::streambuf b;
    std::ostream os(&b);
    os << line << "\r";
    boost::asio::write(serial, b.data());
    os.flush();
    return {SerialError::NONE, "Line written successfully."};
}

SerialDriver::SerialError SerialDriver::setBaudrate(int _baudrate)
{
    if (!serial.is_open())
    {
        std::cerr << "Serial port is not open." << std::endl;
        return {SerialError::PORT_NOT_OPEN, "Serial port is not open."};
    }
    baudrate = _baudrate;
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    return {SerialError::NONE, "Baudrate set successfully."};
}