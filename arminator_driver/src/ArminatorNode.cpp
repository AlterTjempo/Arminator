#include "arminator_driver/ArminatorNode.hpp"
#include <iostream>
#include <map>
#include <algorithm>

using namespace std::placeholders;

ArminatorNode::ArminatorNode(const ServoConfiguration& servo_config, bool test_mode)
    : Node("arminator_driver_node"), servo_config_(servo_config), driver_(servo_config.getSerialPort(), servo_config.baud_rate, test_mode) {
    
    // Create services
    move_servo_service_ = this->create_service<arminator_driver::srv::MoveServo>(
        "move_servo", 
        std::bind(&ArminatorNode::moveServo, this, _1, _2));
    
    estop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "estop", 
        std::bind(&ArminatorNode::estop, this, _1, _2));

    park_service_ = this->create_service<std_srvs::srv::Trigger>(
        "park",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::Park));

    straight_up_service_ = this->create_service<std_srvs::srv::Trigger>(
        "straight_up",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::StraightUp));

    ready_service_ = this->create_service<std_srvs::srv::Trigger>(
        "ready",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::Ready));
    
    RCLCPP_INFO(this->get_logger(), std::string(servo_config.arm_name + " is ready.").c_str() );

    //Dit lijkt niet te werken? 
    try{
        auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
        moveToPredefinedPosition(nullptr, response, Position::Park);
        RCLCPP_INFO(this->get_logger(), "Setup: Moved to park position");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to park position on startup: %s", e.what());
    }
}

ArminatorNode::~ArminatorNode() {}

void ArminatorNode::moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request, 
                              std::shared_ptr<arminator_driver::srv::MoveServo::Response> response) {
    RobotArmDriver::ServoCommand command;
    command.channel = request->servo;

    // Get the calibration offset for this servo channel
    int offset = servo_config_.getOffset(request->servo);

    // Get the physical limits for this servo channel
    auto limits = servo_config_.getLimits(request->servo);
    int min_limit = limits.first;
    int max_limit = limits.second;

    // Calculate the calibrated center position for this servo
    int calibrated_center = 1500 + offset;
    
    // Convert angle in degrees to pulse width in microseconds
    int base_pulse_width = static_cast<int>((request->angle * 2000 / 180.0) + calibrated_center);
    
    // Ensure the pulse width stays within the physical limits for this servo
    if (base_pulse_width < min_limit || base_pulse_width > max_limit) {
        RCLCPP_WARN(this->get_logger(), 
                    "Requested angle %d for servo %d results in out-of-bounds pulse width %dμs (limits: %d to %d). Command not sent.",
                    request->angle, request->servo, base_pulse_width, min_limit, max_limit);
        response->status = 1; // error
        return;
    }
    
    command.pulseWidth = std::max(min_limit, std::min(max_limit, base_pulse_width));
    command.time = request->time;

    if (request->time != 0) {
        command.time = request->time;
    }
    
    RobotArmDriver::RobotArmDriverError error = driver_.sendSingleServoCommand(command);
    if (error.code == RobotArmDriver::RobotArmDriverError::Code::NONE) {
        response->status = 0; // success
        RCLCPP_INFO(this->get_logger(), 
                    "Moved servo %d to angle %d (pulse width: %dμs) over %dms", 
                    request->servo, request->angle, command.pulseWidth, command.time.value_or(0));
    } else {
        response->status = 1; // error
    }
}

void ArminatorNode::estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]], 
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    driver_.stopAllServos();
    response->success = true;
    response->message = "Emergency stop!";
}

void ArminatorNode::moveToPredefinedPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response, 
                                             Position pos) {
    auto it = positions.find(pos);
    if (it == positions.end()) {
        response->success = false;
        response->message = "Unknown position";
        return;
    }

    RobotArmDriver::RobotArmDriverError error = driver_.sendMultiServoCommand(it->second);

    if (error.code == RobotArmDriver::RobotArmDriverError::Code::NONE) {
        response->success = true;
        response->message = "Success";
    } else {
        response->success = false;
        response->message = "Failed";
    }
}