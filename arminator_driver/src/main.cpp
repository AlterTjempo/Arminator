#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"


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

void estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    (void)request; // Suppress unused parameter warning
    driver.stopAllServos();
    response->success = true;
    response->message = "Emergency stop activated";
}

void park(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    (void)request; // Suppress unused parameter warning
    RobotArmDriver::MultiServoCommand parkCommand = {
        {1, 2200, 200, std::nullopt},
        {2, 2000, 200, std::nullopt},
        {3, 800, 200, std::nullopt},
        {4, 1450, 200, std::nullopt},
        {5, 1900, 200, std::nullopt},
    };
    RobotArmDriver::RobotArmDriverError error = driver.sendMultiServoCommand(parkCommand);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->success = true;
        response->message = "Robot parked successfully";
    } else {
        response->success = false;
        response->message = "Failed to park robot";
    }
}

void straight_up(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    (void)request; // Suppress unused parameter warning
    RobotArmDriver::MultiServoCommand straightUpCommand = {
        {1, 1550, 200, std::nullopt},
        {2, 650, 200, std::nullopt},
        {3, 1450, 200, std::nullopt},
        {4, 1450, 200, std::nullopt},
        {5, 2500, 200, std::nullopt},
    };

    RobotArmDriver::RobotArmDriverError error = driver.sendMultiServoCommand(straightUpCommand);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->success = true;
        response->message = "Robot moved to straight up position successfully";
    } else {
        response->success = false;
        response->message = "Failed to move robot to straight up position";
    }
}


void ready(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    (void)request; // Suppress unused parameter warning
    RobotArmDriver::MultiServoCommand readyCommand = {
        {1, 1850, 400, std::nullopt},
        {2, 1650, 400, std::nullopt},
        {3, 1300, 400, std::nullopt},
        {4, 1500, 400, std::nullopt},
        {5, 1000, 400, std::nullopt},
    };

    RobotArmDriver::RobotArmDriverError error = driver.sendMultiServoCommand(readyCommand);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->success = true;
        response->message = "Robot moved to ready position successfully";
    } else {
        response->success = false;
        response->message = "Failed to move robot to ready position";
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

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_service = 
        node->create_service<std_srvs::srv::Trigger>("park", &park);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr straight_up_service = 
        node->create_service<std_srvs::srv::Trigger>("straight_up", &straight_up);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service = 
        node->create_service<std_srvs::srv::Trigger>("ready", &ready);
    
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arminator driver services ready.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
