#pragma once

#include "SerialDriver.hpp"
#include <string>
#include <optional>

class RobotArmDriver
{
public:
    RobotArmDriver(const std::string &port = "/dev/ttyACM0", uint32_t baudrate = 9600);
    ~RobotArmDriver();
    struct ServoCommand
    {
        uint8_t channel;
        uint16_t pulseWidth;
        std::optional<uint16_t> speed;
        std::optional<uint16_t> time;
    };

    struct RobotArmDriverError
    {
        enum Code : int32_t
        {
            NONE = 0,
            SERIAL_ERROR = -1,
            INVALID_COMMAND = -2,
            UNKNOWN_ERROR = -99
        };
        Code code;
        std::string message;
    };

    typedef std::vector<ServoCommand> MultiServoCommand;

    RobotArmDriverError sendSingleServoCommand(const ServoCommand &command);
    RobotArmDriverError sendMultiServoCommand(const MultiServoCommand &commands);
    void queryMovementStatus();
    void queryPulseWidth(uint8_t channel); // is het een channel of iets anderss
    void stopServo(uint8_t channel);
    void stopAllServos();


private:
    std::string toSerialString(const ServoCommand &command);
    std::string toSerialString(const MultiServoCommand &commands);
    RobotArmDriverError validateCommand(const ServoCommand &command);
    RobotArmDriverError validateCommands(const MultiServoCommand &commands);
    SerialDriver serial_driver_;
};
