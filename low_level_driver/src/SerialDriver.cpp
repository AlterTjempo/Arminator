#include "SerialDriver.hpp"
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <thread>

SerialDriver::SerialDriver(const std::string &_port, int _baudrate) : io_(), serial_(io_, _port), port_name_(_port), baudrate_(_baudrate)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(this->baudrate_));

    // arduino defaults
    serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_.set_option(boost::asio::serial_port::character_size(8));
}

SerialDriver::~SerialDriver()
{
    if (serial_.is_open())
    {
        serial_.close();
    }
}

SerialDriver::SerialError SerialDriver::writeLine(const std::string &line)
{
    if (!serial_.is_open())
    {
        std::cerr << "Serial port is not open." << std::endl;
        return {SerialError::PORT_NOT_OPEN, "Serial port is not open."};
    }

    std::cout << "Sending line: " << line << std::endl;

    boost::asio::streambuf b;
    std::ostream os(&b);
    os << line << "\r";
    boost::asio::write(serial_, b.data());
    os.flush();
    return {SerialError::NONE, "Line written successfully."};
}

SerialDriver::SerialError SerialDriver::setBaudrate(uint32_t baudrate)
{
    if (!serial_.is_open())
    {
        std::cerr << "Serial port is not open." << std::endl;
        return {SerialError::PORT_NOT_OPEN, "Serial port is not open."};
    }
    baudrate_ = baudrate;
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    return {SerialError::NONE, "Baudrate set successfully."};
}