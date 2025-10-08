/**
 * @file ArminatorNode.hpp
 * @brief ROS 2 node implementation for the Arminator robot arm driver
 * @author Timo Berendsen, Benjamin Aarsen
 * @date 2025-10-08
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "arminator_driver/srv/move_servo.hpp"
#include "arminator_driver/srv/set_position.hpp"
#include "arminator_driver/ServoConfiguration.hpp"
#include "arminator_driver/positions.h"
#include "RobotArmDriver.hpp"

#include <memory>
#include <string>

/**
 * @class ArminatorNode
 * @brief Main ROS 2 node for controlling the Arminator robot arm
 * 
 * This node provides ROS 2 services for controlling individual servos,
 * setting predefined positions, and emergency stop functionality.
 * It acts as a bridge between ROS 2 and the low-level robot arm driver.
 */
class ArminatorNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for ArminatorNode
     * @param servo_config Configuration parameters for the servo system
     * @param test_mode If true, runs in test mode without actual hardware communication
     */
    ArminatorNode(const ServoConfiguration& servo_config,
                  bool test_mode = false);
    
    /**
     * @brief Destructor for ArminatorNode
     */
    ~ArminatorNode();
private:
    /**
     * @name Service Callbacks
     * @brief ROS 2 service callback functions
     * @{
     */
    
    /**
     * @brief Service callback for moving a single servo
     * @param request Service request containing servo ID, position, and speed
     * @param response Service response indicating success/failure
     */
    void moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request,
                   std::shared_ptr<arminator_driver::srv::MoveServo::Response> response);
    
    
    /**
     * @brief Emergency stop service callback
     * @param request Service request (empty for trigger service)
     * @param response Service response indicating success/failure
     */
    void estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Move to a predefined position
     * @param request Service request (empty for trigger service)
     * @param response Service response indicating success/failure
     * @param pos The predefined position to move to
     */
    void moveToPredefinedPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                  Position pos);
    
    /// @}

    /**
     * @name Private Members
     * @brief Internal state and configuration
     * @{
     */
    ServoConfiguration servo_config_;   ///< Servo configuration parameters
    RobotArmDriver driver_;            ///< Low-level robot arm driver instance
    /// @}
    
    /**
     * @name ROS 2 Services
     * @brief Service interfaces for robot control
     * @{
     */
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service_;      ///< Single servo movement service
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service_;  ///< Multi-servo position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;                    ///< Emergency stop service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_service_;                     ///< Park position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr straight_up_service_;              ///< Straight up position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;                    ///< Ready position service
    /// @}
};