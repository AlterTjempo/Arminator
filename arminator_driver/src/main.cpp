#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"


#include <iostream>
#include <map>
#include <algorithm>
#include "RobotArmDriver.hpp"

RobotArmDriver driver("/dev/ttyUSB1", 115200);

// Servo calibration offsets - these adjust the center position (1500μs) for each servo
const std::map<int, int> SERVO_OFFSETS = {
    {1, 50},   // Channel 1: +50 offset (center at 1550)
    {2, -50},  // Channel 2: +150 offset (center at 650)
    {3, -50},  // Channel 3: -50 offset (center at 1450)
    {4, -50},  // Channel 4: -50 offset (center at 1450)
    {5, 400}   // Channel 5: +400 offset (center at 1900)
};

// Physical limits for each servo channel based on mechanical constraints
const std::map<int, std::pair<int, int>> SERVO_LIMITS = {
    {1, {850, 1900}},   // Channel 1 (Shoulder): 850μs (naar voren) to 1900μs (max uitslag)
    {2, {650, 1450}},   // Channel 2 (Elbow): 650μs (recht omhoog) to 1450μs (90 graden)
    {3, {550, 2450}},   // Channel 3 (Wrist): 550μs (omlaag) to 2450μs (omhoog)
    {4, {550, 2450}},   // Channel 4 (Wrist Rotate): 550μs (verticaal links) to 2450μs (verticaal rechts)
    {5, {1000, 2500}}   // Channel 5 (Gripper): 1000μs (volledig open) to 2500μs (volledig dicht)
};

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
    // Map -90 to +90 degrees to the available range for this specific servo
    int available_range = max_limit - min_limit;
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
                    request->servo, request->angle, command.pulseWidth, command.time);
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
    // Park mode values from documentation (channel 0 is defect, so skipped)
    RobotArmDriver::MultiServoCommand parkCommand = {
        {1, 1900, 200, std::nullopt}, // Channel 1: 1900μs
        {2, 1950, 200, std::nullopt}, // Channel 2: 1950μs
        {3, 650, 200, std::nullopt},  // Channel 3: 650μs
        {4, 1500, 200, std::nullopt}, // Channel 4: 1500μs (maar should this be 1450 for horizontal?)
        {5, 1900, 200, std::nullopt}, // Channel 5: 1900μs (half open - belangrijk!)
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
    // Straight up position using calibrated values from offset table
    RobotArmDriver::MultiServoCommand straightUpCommand = {
        {1, 1550, 200, std::nullopt}, // Channel 1: Recht omhoog (1500 + 50 offset)
        {2, 650, 200, std::nullopt},  // Channel 2: Recht omhoog (500 + 150 offset)
        {3, 1450, 200, std::nullopt}, // Channel 3: Recht (1500 - 50 offset)
        {4, 1450, 200, std::nullopt}, // Channel 4: Horizontaal (1500 - 50 offset)
        {5, 2500, 200, std::nullopt}, // Channel 5: Volledig dicht (2500 + 0 offset)
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
    // Ready mode values from documentation (channel 0 is defect, so skipped)
    RobotArmDriver::MultiServoCommand readyCommand = {
        {1, 1850, 400, std::nullopt}, // Channel 1: 1850μs
        {2, 1650, 400, std::nullopt}, // Channel 2: 1650μs
        {3, 1300, 400, std::nullopt}, // Channel 3: 1300μs
        {4, 1500, 400, std::nullopt}, // Channel 4: 1500μs
        {5, 1000, 400, std::nullopt}, // Channel 5: 1000μs (helemaal open)
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
