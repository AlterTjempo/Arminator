#include "rclcpp/rclcpp.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"
#include "arminator_driver/srv/estop.hpp"


#include <iostream>
#include "RobotArmDriver.hpp"

void moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request, std::shared_ptr<arminator_driver::srv::MoveServo::Response> response){
    // RobotArmDriver driver("/dev/ttyUSB0", 115200);
    RobotArmDriver::ServoCommand command;
    command.channel = request->channel;
    command.pulseWidth = request->pulse_width;
    if(request->speed != 0){
        command.speed = request->speed;
    }
    if(request->time != 0){
        command.time = request->time;
    }
    RobotArmDriver::RobotArmDriverError error = driver.sendSingleServoCommand(command);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->success = true;
        response->message = "Command sent successfully";
    } else {
        response->success = false;
        response->message = "Error: " + error.message;
    }

}


int main(int argc [[maybe_unused]], char const *argv[] [[maybe_unused]]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr service = node->create_service<arminator_driver::srv::MoveServo>("move_servo", &moveServo);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
