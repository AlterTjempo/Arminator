/**
 * @file ServoConfiguration.hpp
 * @brief Configuration management for robot arm servo parameters
 * @author Timo Berendsen, Benjamin Aarsen
 * @date 2025-10-08
 */

#pragma once
#include <map>
#include <utility>
#include <string>

/**
 * @struct ServoConfiguration
 * @brief Configuration structure for robot arm servo parameters and limits
 * 
 * This structure holds all configuration parameters needed to operate the robot arm,
 * including serial communication settings, servo-specific offsets and limits.
 */
struct ServoConfiguration {
    std::string serial_port;                        ///< Serial port device path
    int baud_rate;                                  ///< Communication baud rate
    std::string arm_name;                           ///< Human-readable name for the arm
    int servo_count;                                ///< Number of servos in the arm
    std::map<int, int> offsets;                     ///< Per-servo position offsets (channel -> offset)
    std::map<int, std::pair<int, int>> limits;      ///< Per-servo position limits (channel -> {min, max})
    
    /**
     * @brief Default constructor with sensible defaults
     * 
     * Initializes the configuration with default values suitable for most
     * standard robot arm setups.
     */
    ServoConfiguration() : 
        serial_port("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06DKH3-if00-port0"),
        baud_rate(115200),
        arm_name("Unknown"),
        servo_count(6) {}
    
    /**
     * @brief Get offset for a servo channel
     * @param channel The servo channel number
     * @return The offset value for the channel, or 0 if not found
     */
    int getOffset(int channel) const;
    
    /**
     * @brief Get position limits for a servo channel
     * @param channel The servo channel number
     * @return A pair containing {min, max} limits, or default limits if not found
     */
    std::pair<int, int> getLimits(int channel) const;
    
    /**
     * @brief Get the full serial port path
     * @return The complete serial port device path
     */
    std::string getSerialPort() const;
    
    /**
     * @brief Load configuration from a file
     * @param path Path to the configuration file
     * @return ServoConfiguration object loaded from the file
     * @throws std::runtime_error if the file cannot be loaded or parsed
     */
    static ServoConfiguration loadFromFile(const std::string& path);
};