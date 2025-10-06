#include "rclcpp/rclcpp.hpp"
#include "arminator_driver/ArminatorNode.hpp"
#include "arminator_driver/ServoConfiguration.hpp"

#include <iostream>


int main(int argc, char const *argv[]) {
    ServoConfiguration servo_config;
    try {
        servo_config = ServoConfiguration::loadFromFile("./arminator_driver/configs/servo_config.ini");
    } catch (const std::exception& err) {
        std::cerr << "Error loading servo configuration: " << err.what() << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto arminator_node = std::make_shared<ArminatorNode>(servo_config);
    rclcpp::spin(arminator_node);
    rclcpp::shutdown();
    return 0;
}
