#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"


#include <iostream>
#include "RobotArmDriver.hpp"
#include "arminator_driver/positions.h"


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

void estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    (void)request; // Suppress unused parameter warning
    driver.stopAllServos();
    response->success = true;
    response->message = "Emergency stop!";
}

void moveToPredefinedPosition(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
    std::shared_ptr<std_srvs::srv::Trigger::Response> response, Position pos ){
        
    auto it = positions.find(pos);
    if (it == positions.end()) {
        response->success = false;
        response->message = "Unknown position";
        return;
    }

    RobotArmDriver::RobotArmDriverError error = driver.sendMultiServoCommand(it->second);

    if (error.code == RobotArmDriver::RobotArmDriverError::Code::NONE) {
        response->success = true;
        response->message = "Success";
    } else {
        response->success = false;
        response->message = "Failed";
    }
}



int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arminator_driver_node");
    
    // Create services
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service = 
        node->create_service<arminator_driver::srv::MoveServo>("move_servo", &moveServo);
    
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service = 
        node->create_service<arminator_driver::srv::SetPosition>("set_position", &setPosition);
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service = 
        node->create_service<std_srvs::srv::Trigger>("estop", &estop);

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_service = 
    //     node->create_service<std_srvs::srv::Trigger>("park", &park);

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr straight_up_service = 
    //     node->create_service<std_srvs::srv::Trigger>("straight_up", &straight_up);

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service = 
    //     node->create_service<std_srvs::srv::Trigger>("ready", &ready);

    using namespace std::placeholders;
    
    auto park_service = node->create_service<std_srvs::srv::Trigger>("park",
    std::bind(moveToPredefinedPosition, _1, _2, Position::Park));

    auto straight_up_service = node->create_service<std_srvs::srv::Trigger>("straight_up",
        std::bind(moveToPredefinedPosition, _1, _2, Position::StraightUp));

    auto ready_service = node->create_service<std_srvs::srv::Trigger>("ready",
        std::bind(moveToPredefinedPosition, _1, _2, Position::Ready));
    
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arminator driver services ready.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
