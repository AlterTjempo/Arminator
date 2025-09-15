#include <iostream>
#include "RobotArmDriver.hpp"

RobotArmDriver::RobotArmDriver(const std::string &port, uint32_t baudrate)
    : serial_driver_(port, baudrate)
{
}

RobotArmDriver::~RobotArmDriver()
{
}

RobotArmDriver::RobotArmDriverError RobotArmDriver::sendSingleServoCommand(const ServoCommand &command)
{
    if (auto error = validateCommand(command); error.code != RobotArmDriverError::NONE)
    {
        std::cerr << "Invalid command: " << error.message << std::endl;
        return error;
    }
    std::string commandStr = toSerialString(command);
    auto error = serial_driver_.writeLine(commandStr);

    if (error.code != SerialDriver::SerialError::NONE)
    {
        std::cerr << "Error sending command: " << error.message << std::endl;
        return {RobotArmDriverError::SERIAL_ERROR, error.message};
    }

    return {RobotArmDriverError::NONE, ""};
}

RobotArmDriver::RobotArmDriverError RobotArmDriver::sendMultiServoCommand(const MultiServoCommand &commands)
{
    if (auto error = validateCommands(commands); error.code != RobotArmDriverError::NONE)
    {
        std::cerr << "Invalid command in multi-command: " << error.message << std::endl;
        return error;
    }
    
    std::string commandStr = toSerialString(commands);
    auto error = serial_driver_.writeLine(commandStr);

    if (error.code != SerialDriver::SerialError::NONE)
    {
        std::cerr << "Error sending command: " << error.message << std::endl;
        return {RobotArmDriverError::SERIAL_ERROR, error.message};
    }

    return {RobotArmDriverError::NONE, ""};
}

std::string RobotArmDriver::toSerialString(const ServoCommand &command)
{
    std::string cmd = "#" + std::to_string(command.channel) + "P" + std::to_string(command.pulseWidth);
    if (command.speed.has_value())
    {
        cmd += "S" + std::to_string(command.speed.value());
    }
    if (command.time.has_value())
    {
        cmd += "T" + std::to_string(command.time.value());
    }
    return cmd;
}

std::string RobotArmDriver::toSerialString(const MultiServoCommand &commands)
{
    std::string cmd;
    for (const auto &command : commands)
    {
        if (!cmd.empty())
        {
            cmd += " "; // Separate multiple commands with a space
        }
        cmd += toSerialString(command);
    }
    return cmd;
}

RobotArmDriver::RobotArmDriverError RobotArmDriver::validateCommand(const ServoCommand &command)
{
    // it is not allowed to have both speed and time set
    if (command.speed.has_value() && command.time.has_value())
    {
        return {RobotArmDriverError::INVALID_COMMAND, "Both speed and time cannot be set simultaneously."};
    }

    // pulse width must be between 500 and 2500
    if (command.pulseWidth < 500 || command.pulseWidth > 2500)
    {
        return {RobotArmDriverError::INVALID_COMMAND, "Pulse width must be between 500 and 2500."};
    }
    return {RobotArmDriverError::NONE, ""};
}

RobotArmDriver::RobotArmDriverError RobotArmDriver::validateCommands(const MultiServoCommand &commands)
{
    for (const auto &command : commands)
    {
        const auto &error = validateCommand(command);
        if (error.code != RobotArmDriverError::NONE)
        {
            return error;
        }
    }
    return {RobotArmDriverError::NONE, ""};
}
