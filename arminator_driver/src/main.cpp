#include <iostream>
#include "RobotArmDriver.hpp"

int main(int argc, char const *argv[])
{
    // Suppress unused parameter warnings
    (void)argc;
    (void)argv;
    RobotArmDriver driver("/dev/ttyUSB0", 115200);

    driver.sendSingleServoCommand({5, 2500, std::nullopt, std::nullopt});
    std::cout << "Hello, Arminator!" << std::endl;
    return 0;
}
