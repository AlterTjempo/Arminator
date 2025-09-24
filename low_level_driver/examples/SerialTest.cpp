#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "RobotArmDriver.hpp"

//Arguments: port, baudrate, angle, time_ms (optional)
#define NUMBER_OF_ARGUMENTS 4

int main(int argc, char const *argv[]){
	if(argc < NUMBER_OF_ARGUMENTS){
		std::cerr << "Please provide a port, baudrate, angle (time optional)" << '\n';
		return -1;
	}

	int status = 0;
	std::string port = argv[1];
	int baudrate = std::stoi(argv[2]);
	int angle = std::stoi(argv[3]);
	int time_ms = -1;

	// //Make a servo controller object
	// RobotArmDriver controller(port, baudrate);

	// if(argc == 5){
	// 	time_ms = std::stoi(argv[4]);
	// 	status = controller.setTimedAngle(angle, time_ms);
	// }else{
	// 	status = controller.setAngle(angle);
	// }
	// std::cout << "Exited with code: " << status << '\n';
	return status;
}
