/**
 * @file SerialDriver.hpp
 * @brief Low-level serial communication driver for robot arm control
 * @author Timo Berendsen, Benjamin Aarsen
 * @date 2025-10-08
 */

#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <cstdint>

/**
 * @class SerialDriver
 * @brief Low-level serial communication interface
 * 
 * This class provides a simple interface for serial communication using
 * Boost.Asio. It handles reading and writing of text and binary data
 * over a serial connection.
 */
class SerialDriver
{
public:
    /**
     * @brief Constructor for SerialDriver
     * @param port Serial port device path (default: "/dev/ttyUSB0")
     * @param baudrate Communication baud rate (default: 115200)
     */
    SerialDriver(const std::string &port = "/dev/ttyUSB0", int baudrate = 115200);
    
    /**
     * @brief Destructor for SerialDriver
     */
    ~SerialDriver();

    /**
     * @struct SerialError
     * @brief Error structure for serial communication operations
     */
    struct SerialError
    {
        /**
         * @enum Code
         * @brief Error code enumeration for serial operations
         */
        enum Code : int32_t
        {
            NONE = 0,               ///< No error occurred
            PORT_NOT_OPEN = -1,     ///< Serial port is not open
            UNKNOWN_ERROR = -99     ///< Unknown error occurred
        };
        Code code;              ///< Error code
        std::string message;    ///< Human-readable error message
    };

    /**
     * @name Communication Methods
     * @brief Serial read/write operations
     * @{
     */
    
    /**
     * @brief Write a line of text to the serial port
     * @param line The string to write (newline will be appended)
     * @return SerialError indicating success or failure
     */
    SerialError writeLine(const std::string &line);
    
    /**
     * @brief Read a line of text from the serial port
     * @param line Reference to string that will contain the read data
     * @return SerialError indicating success or failure
     */
    SerialError readLine(std::string &line);

    /**
     * @brief Read binary data from the serial port
     * @param line Reference to vector that will contain the read bytes
     * @return SerialError indicating success or failure
     * @note This method reads pulse width data as numbers between 50 and 250
     *       (actual pulse width = value / 1000)
     */
    SerialDriver::SerialError readLine(std::vector<uint8_t> &line);
    
    /// @}
    
    /**
     * @name Configuration Methods
     * @brief Serial port configuration
     * @{
     */
    
    /**
     * @brief Set the baud rate for serial communication
     * @param baudrate New baud rate value
     * @return SerialError indicating success or failure
     */
    SerialError setBaudrate(uint32_t baudrate);
    
    /**
     * @brief Get the current baud rate
     * @return Current baud rate value
     */
    uint32_t getBaudrate() const { return baudrate_; }
    
    /// @}

private:
    /**
     * @name Private Members
     * @brief Internal state and communication objects
     * @{
     */
    boost::asio::io_context io_;        ///< Boost.Asio I/O context
    boost::asio::serial_port serial_;   ///< Serial port object

    std::string port_name_;             ///< Serial port device name
    uint32_t baudrate_;                 ///< Current baud rate setting
    /// @}
};
