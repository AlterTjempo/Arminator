#include "robot_arm_driver/RobotArmDriver.hpp"
#include <iostream>

RobotArmDriver::RobotArmDriver(const std::string &port, int baudrate)
    : serial_driver_(port, baudrate)
{
}

RobotArmDriver::~RobotArmDriver()
{
}

int RobotArmDriver::setAngle(int angle)
{
    std::string command = "turn " + std::to_string(angle);
    return serial_driver_.writeLine(command);
}

int RobotArmDriver::setTimedAngle(int angle, int time_ms)
{
    //Disallow if the time / angle is more than 2.5, avoiding division by zero.
    if (angle != 0 && time_ms / angle < 2.5) {
        std::cerr << "Movement is too fast." << '\n';
        return -1;
    }

    std::string command = "timed_turn " + std::to_string(time_ms) + " " + std::to_string(angle);
    return serial_driver_.writeLine(command);
}