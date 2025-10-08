/**
 * @file RobotArmDriver.hpp
 * @brief High-level driver interface for robot arm control
 * @author Timo Berendsen, Benjamin Aarsen
 * @date 2025-10-08
 */

#pragma once

#include <string>
#include <optional>
#include <vector>
#include <memory>

// Forward declaration
class SerialDriver;

/**
 * @class RobotArmDriver
 * @brief High-level interface for controlling a serial-connected robot arm
 * 
 * This class provides a high-level API for sending commands to a robot arm
 * via serial communication. It supports both single servo commands and
 * multi-servo coordinated movements.
 */
class RobotArmDriver
{
public:
    /**
     * @brief Constructor for RobotArmDriver
     * @param port Serial port device path (e.g., "/dev/ttyUSB0")
     * @param baudrate Communication baud rate (default: 115200)
     * @param test_mode If true, runs in test mode without actual serial communication
     */
    RobotArmDriver(const std::string &port, uint32_t baudrate = 115200, bool test_mode = false);
    
    /**
     * @brief Destructor for RobotArmDriver
     */
    ~RobotArmDriver();
    
    /**
     * @struct ServoCommand
     * @brief Command structure for individual servo control
     */
    struct ServoCommand
    {
        uint8_t channel;                        ///< Servo channel/ID number
        uint16_t pulseWidth;                    ///< Target pulse width in microseconds
        std::optional<uint16_t> speed;          ///< Movement speed (optional)
        std::optional<uint16_t> time;           ///< Movement time duration (optional)
    };

    /**
     * @struct RobotArmDriverError
     * @brief Error structure for driver operation results
     */
    struct RobotArmDriverError
    {
        /**
         * @enum Code
         * @brief Error code enumeration
         */
        enum Code : int32_t
        {
            NONE = 0,                ///< No error
            SERIAL_ERROR = -1,       ///< Serial communication error
            INVALID_COMMAND = -2,    ///< Invalid command parameters
            UNKNOWN_ERROR = -99      ///< Unknown error occurred
        };
        Code code;              ///< Error code
        std::string message;    ///< Human-readable error message
    };

    /// @brief Type alias for multiple servo commands
    typedef std::vector<ServoCommand> MultiServoCommand;

    /**
     * @name Public Interface Methods
     * @brief Main API for robot arm control
     * @{
     */
    
    /**
     * @brief Send a command to a single servo
     * @param command ServoCommand containing target parameters
     * @return RobotArmDriverError indicating success or failure
     */
    RobotArmDriverError sendSingleServoCommand(const ServoCommand &command);
    
    /**
     * @brief Send coordinated commands to multiple servos
     * @param commands Vector of ServoCommand structures
     * @return RobotArmDriverError indicating success or failure
     */
    RobotArmDriverError sendMultiServoCommand(const MultiServoCommand &commands);
    
    /**
     * @brief Query the current movement status of all servos
     */
    void queryMovementStatus();
    
    /**
     * @brief Query the current pulse width of a specific servo
     * @param channel Servo channel number to query
     */
    void queryPulseWidth(uint8_t channel);
    
    /**
     * @brief Stop a specific servo immediately
     * @param channel Servo channel number to stop
     */
    void stopServo(uint8_t channel);
    
    /**
     * @brief Emergency stop - halt all servo movements immediately
     */
    void stopAllServos();
    
    /// @}

private:
    /**
     * @name Private Helper Methods
     * @brief Internal utility functions
     * @{
     */
    
    /**
     * @brief Convert a single servo command to serial string format
     * @param command The servo command to convert
     * @return Serial command string
     */
    std::string toSerialString(const ServoCommand &command);
    
    /**
     * @brief Convert multiple servo commands to serial string format
     * @param commands The servo commands to convert
     * @return Serial command string
     */
    std::string toSerialString(const MultiServoCommand &commands);
    
    /**
     * @brief Validate a single servo command
     * @param command The command to validate
     * @return RobotArmDriverError indicating validation result
     */
    RobotArmDriverError validateCommand(const ServoCommand &command);
    
    /**
     * @brief Validate multiple servo commands
     * @param commands The commands to validate
     * @return RobotArmDriverError indicating validation result
     */
    RobotArmDriverError validateCommands(const MultiServoCommand &commands);
    
    /// @}
    
    /// @brief Serial communication driver instance
    std::unique_ptr<SerialDriver> serial_driver_;
};
