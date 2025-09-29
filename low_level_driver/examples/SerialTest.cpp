#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "RobotArmDriver.hpp"
#include <thread>

// Arguments: port, baudrate, pulsewidth, time_ms (optional)
#define MIN_NUMBER_OF_ARGUMENTS 5

int main(int argc, char const *argv[])
{
	if (argc < MIN_NUMBER_OF_ARGUMENTS)
	{
		std::cerr << "Please provide a port, baudrate, channel, pulsewidth, time(optional), speed(optional)" << '\n';
		return -1;
	}

	int status = 0;
	std::string port = argv[1];
	int baudrate = std::stoi(argv[2]);
	int channel = std::stoi(argv[3]);
	int pulsewidth = std::stoi(argv[4]);

	RobotArmDriver driver(port, baudrate);

	RobotArmDriver::ServoCommand command;

	if (argc == 6)
	{
		int time = std::stoi(argv[5]);
		command.time = time;
	}
	if (argc == 7)
	{
		int speed = std::stoi(argv[6]);
		command.speed = speed;
	}

	command.channel = channel;
	command.pulseWidth = pulsewidth;

	auto error = driver.sendSingleServoCommand(command);
	if (error.code != RobotArmDriver::RobotArmDriverError::NONE)
	{
		std::cerr << "Error sending command: " << error.message << '\n';
		status = -1;
	}
	// std::this_thread::sleep_for(std::chrono::seconds(2));
	driver.queryMovementStatus();
	driver.queryPulseWidth(channel);	

	// driver.stopAllServos();
	return status;
}
