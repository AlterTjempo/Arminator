#ifndef ARMINATOR_NODE_HPP
#define ARMINATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"
#include "arminator_driver/ServoConfiguration.hpp"
#include "arminator_driver/positions.h"
#include "RobotArmDriver.hpp"

#include <memory>
#include <string>

class ArminatorNode : public rclcpp::Node {
public:
    ArminatorNode(const ServoConfiguration& servo_config,
                  bool test_mode = false);
    
    ~ArminatorNode();
private:
    // Service callbacks
    void moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request,
                   std::shared_ptr<arminator_driver::srv::MoveServo::Response> response);
    
    void setPosition(const std::shared_ptr<arminator_driver::srv::SetPosition::Request> request,
                     std::shared_ptr<arminator_driver::srv::SetPosition::Response> response);
    
    void estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void moveToPredefinedPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                  Position pos);

    // Private members
    ServoConfiguration servo_config_;
    RobotArmDriver driver_;
    
    // Services
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service_;
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr straight_up_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
};

#endif // ARMINATOR_NODE_HPP