#pragma once

#include "robot_arm_driver/SerialDriver.hpp"
#include <string>

class RobotArmDriver
{
public:
    RobotArmDriver(const std::string &port = "/dev/ttyACM0", int baudrate = 9600);
    ~RobotArmDriver();

    int setAngle(int angle);
    int setTimedAngle(int angle, int time_ms);

private:
    SerialDriver serial_driver_;
};
