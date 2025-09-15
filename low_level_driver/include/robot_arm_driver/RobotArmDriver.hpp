#pragma once

#include "robot_arm_driver/SerialDriver.hpp"
#include <string>

class RobotArmDriver
{
public:
    struct ServoCommand {
        uint8_t channel;
        uint16_t pulseWidth;
        uint16_t speed;
        uint16_t time;
    };

    typedef std::vector<ServoCommand> MultiServoCommand;

private:
    SerialDriver serial_driver_;
};
