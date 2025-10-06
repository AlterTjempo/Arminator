#include <iostream>
#include "RobotArmDriver.hpp"
#include "SerialDriver.hpp"
#include <thread>

RobotArmDriver::RobotArmDriver(const std::string &port, uint32_t baudrate, bool test_mode) {
    //We can not use the initializer list here because of the test_mode option
     if (!test_mode) {
            serial_driver_ = std::make_unique<SerialDriver>(port, baudrate);
    } else {
        //fake driver for testing without hardware
        serial_driver_ = nullptr;
        std::cout << "#! Test mode enabled: No hardware connection established." << '\n';
        std::cout << "#! Running commands will throw segfaults!" << std::endl;
    }
}

RobotArmDriver::~RobotArmDriver() {
}

RobotArmDriver::RobotArmDriverError RobotArmDriver::sendSingleServoCommand(const ServoCommand &command)
{
    if (auto error = validateCommand(command); error.code != RobotArmDriverError::NONE)
    {
        std::cerr << "Invalid command: " << error.message << std::endl;
        return error;
    }
    std::string commandStr = toSerialString(command);
    auto error = serial_driver_->writeLine(commandStr);

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
    auto error = serial_driver_->writeLine(commandStr);

    if (error.code != SerialDriver::SerialError::NONE)
    {
        std::cerr << "Error sending command: " << error.message << std::endl;
        return {RobotArmDriverError::SERIAL_ERROR, error.message};
    }

    return {RobotArmDriverError::NONE, ""};
}


// Stopping:  
void RobotArmDriver::stopServo(uint8_t channel) {
    serial_driver_->writeLine("STOP" + std::to_string(channel));
}


void RobotArmDriver::stopAllServos()
{
    for (uint8_t channel = 0; channel < 6; ++channel) 
    {
        stopServo(channel);
    }
}

void RobotArmDriver::queryMovementStatus() {
    std::string commandStr = "Q";
    auto error = serial_driver_->writeLine(commandStr);
    if (error.code != SerialDriver::SerialError::NONE) {
        std::cerr << "Error querying movement status: " << error.message << std::endl;
        return;
    }

    // Print out the response from the serial port
    std::string response;
    error = serial_driver_->readLine(response);
    if (error.code != SerialDriver::SerialError::NONE) {
        std::cerr << "Error reading response: " << error.message << std::endl;
    } else {
        std::cout << "Pulse width response: " << response<< std::endl;
    }
}

void RobotArmDriver::queryPulseWidth(uint8_t channel) {
    std::string commandStr = "QP" + std::to_string(channel);
    auto error = serial_driver_->writeLine(commandStr);
    if (error.code != SerialDriver::SerialError::NONE){
        std::cerr << "Error querying pulse width: " << error.message << std::endl;
    }

    //Print out the response from the serial port
    std::vector<uint8_t> response;
    error = serial_driver_->readLine(response);
    if (error.code != SerialDriver::SerialError::NONE) {
        std::cerr << "Error reading response: " << error.message << std::endl;
    } else {
        std::cout << "Movement status response: ";
        for (const auto &byte : response) {
            std::cout << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
    }
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
