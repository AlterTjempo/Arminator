#include "rclcpp/rclcpp.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"
#include "arminator_driver/srv/estop.hpp"


#include <iostream>
#include "RobotArmDriver.hpp"

RobotArmDriver driver("/dev/ttyUSB0", 115200);

void moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request, std::shared_ptr<arminator_driver::srv::MoveServo::Response> response){
    RobotArmDriver::ServoCommand command;
    command.channel = request->servo;
    command.pulseWidth = request->angle; // assuming angle maps to pulse width
    if(request->time != 0){
        command.time = request->time;
    }
    RobotArmDriver::RobotArmDriverError error = driver.sendSingleServoCommand(command);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->status = 0; // success
    } else {
        response->status = 1; // error
    }
}

void setPosition(const std::shared_ptr<arminator_driver::srv::SetPosition::Request> request, std::shared_ptr<arminator_driver::srv::SetPosition::Response> response){
    (void)request; // Suppress unused parameter warning
    // TODO: Implement position setting logic
    // For now, just return success
    response->status = 0;
}

void estop(const std::shared_ptr<arminator_driver::srv::Estop::Request> request, std::shared_ptr<arminator_driver::srv::Estop::Response> response){
    (void)request; // Suppress unused parameter warning
    // TODO: Implement emergency stop logic
    // For now, just return success
    response->status = 0;
}

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arminator_driver_node");
    
    // Create services
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service = 
        node->create_service<arminator_driver::srv::MoveServo>("move_servo", &moveServo);
    
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service = 
        node->create_service<arminator_driver::srv::SetPosition>("set_position", &setPosition);
    
    rclcpp::Service<arminator_driver::srv::Estop>::SharedPtr estop_service = 
        node->create_service<arminator_driver::srv::Estop>("estop", &estop);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arminator driver services ready.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
