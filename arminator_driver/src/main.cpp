#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"
#include "arminator_driver/servoConfig.hpp"
#include "arminator_driver/positions.h"

#include <iostream>
#include <map>
#include <algorithm>
#include "RobotArmDriver.hpp"


using namespace std::placeholders; //in service callbacks

RobotArmDriver driver("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06DKH3-if00-port0", 115200);
// RobotArmDriver driver("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06DKH3-if00-port0", 115200, true); //for testing without hardware


void moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request, std::shared_ptr<arminator_driver::srv::MoveServo::Response> response){
    RobotArmDriver::ServoCommand command;
    command.channel = request->servo;

    // Get the calibration offset for this servo channel
    int offset = 0;
    auto offset_it = SERVO_OFFSETS.find(request->servo);
    if (offset_it != SERVO_OFFSETS.end()) {
        offset = offset_it->second;
    }

    // Get the physical limits for this servo channel
    int min_limit = 500;  // Default generic limits
    int max_limit = 2500;
    auto limits_it = SERVO_LIMITS.find(request->servo);
    if (limits_it != SERVO_LIMITS.end()) {
        min_limit = limits_it->second.first;
        max_limit = limits_it->second.second;
    }

    // Calculate the calibrated center position for this servo
    int calibrated_center = 1500 + offset;
    
    // Convert angle in degrees to pulse width in microseconds
    int base_pulse_width = static_cast<int>((request->angle * 2000 / 180.0) + calibrated_center);
    
    // Ensure the pulse width stays within the physical limits for this servo

    if (base_pulse_width < min_limit || base_pulse_width > max_limit) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Requested angle %d for servo %d results in out-of-bounds pulse width %dμs (limits: %d to %d). Command not sent.",
                    request->angle, request->servo, base_pulse_width, min_limit, max_limit);
        response->status = 1; // error
        return;
    }
    
    command.pulseWidth = std::max(min_limit, std::min(max_limit, base_pulse_width));
    
    command.time = request->time;

    if(request->time != 0){
        command.time = request->time;
    }
    RobotArmDriver::RobotArmDriverError error = driver.sendSingleServoCommand(command);
    if(error.code == RobotArmDriver::RobotArmDriverError::Code::NONE){
        response->status = 0; // success
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moved servo %d to angle %d (pulse width: %dμs) over %dms", 
                    request->servo, request->angle, command.pulseWidth, command.time.value_or(0));
    } else {
        response->status = 1; // error
    }
}

void setPosition(const std::shared_ptr<arminator_driver::srv::SetPosition::Request> request [[maybe_unused]], std::shared_ptr<arminator_driver::srv::SetPosition::Response> response){
    // TODO: Implement position setting logic
    // For now, just return success
    response->status = 0;
}

void estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]], std::shared_ptr<std_srvs::srv::Trigger::Response> response){
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
    try{
        load_servo_config("./arminator_driver/configs/servo_config.ini");
    }catch (const std::exception& err){
        std::cerr << "Error loading servo configuration: " << err.what() << std::endl;
        return 1;
    }


    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arminator_driver_node");
    
    // Create services
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service = 
        node->create_service<arminator_driver::srv::MoveServo>("move_servo", &moveServo);
    
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service = 
        node->create_service<arminator_driver::srv::SetPosition>("set_position", &setPosition);
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service = 
        node->create_service<std_srvs::srv::Trigger>("estop", &estop);

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
