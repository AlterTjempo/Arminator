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
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <variant>

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
     * @brief Command types that can be queued
     */
    struct SingleServoCommandInfo {
        RobotArmDriver::ServoCommand command;
        std::function<void(bool)> callback;
    };

    struct MultiServoCommandInfo {
        RobotArmDriver::MultiServoCommand commands;
        std::function<void(bool)> callback;
    };

    using QueuedCommand = std::variant<SingleServoCommandInfo, MultiServoCommandInfo>;

    /**
     * @brief Structure to hold command with timing information
     */
    struct TimedCommand {
        QueuedCommand command;
        std::chrono::milliseconds execution_time;
    };
    /**
     * @name Command Queue Management
     * @brief Methods for managing the command queue
     * @{
     */
    
    /**
     * @brief Add a single servo command to the queue
     * @param command The servo command to queue
     * @param callback Callback function to call when command completes
     */
    void queueSingleServoCommand(const RobotArmDriver::ServoCommand& command, 
                                std::function<void(bool)> callback);
    
    /**
     * @brief Add a multi-servo command to the queue
     * @param commands The servo commands to queue
     * @param callback Callback function to call when command completes
     */
    void queueMultiServoCommand(const RobotArmDriver::MultiServoCommand& commands,
                               std::function<void(bool)> callback);
    
    /**
     * @brief Clear all commands from the queue
     */
    void clearCommandQueue();
    
    /**
     * @brief Process the command queue (runs in separate thread)
     */
    void processCommandQueue();
    
    /**
     * @brief Execute a single command from the queue
     * @param timed_command The command to execute
     * @return true if successful, false otherwise
     */
    bool executeCommand(const TimedCommand& timed_command);
    
    /**
     * @brief Execute a single servo command
     * @param cmd_info The single servo command information
     * @return true if successful, false otherwise
     */
    bool executeSingleServoCommand(const SingleServoCommandInfo& cmd_info);
    
    /**
     * @brief Execute a multi-servo command
     * @param cmd_info The multi-servo command information
     * @return true if successful, false otherwise
     */
    bool executeMultiServoCommand(const MultiServoCommandInfo& cmd_info);
    
    /**
     * @brief Calculate the maximum execution time from a list of servo commands
     * @param commands The commands to analyze
     * @return Maximum execution time in milliseconds
     */
    std::chrono::milliseconds calculateMaxExecutionTime(const RobotArmDriver::MultiServoCommand& commands);
    
    /// @}

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
     * @brief Reset emergency stop service callback
     * @param request Service request (empty for trigger service)
     * @param response Service response indicating success/failure
     */
    void resetEstop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Get queue status service callback
     * @param request Service request (empty for trigger service)
     * @param response Service response with queue status information
     */
    void getQueueStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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
    
    // Command queue members
    std::queue<TimedCommand> command_queue_;        ///< Queue of pending commands
    std::mutex queue_mutex_;                        ///< Mutex for thread-safe queue access
    std::thread queue_thread_;                      ///< Thread for processing the command queue
    std::atomic<bool> queue_running_;               ///< Flag to control queue processing thread
    std::atomic<bool> emergency_stop_active_;      ///< Flag indicating if emergency stop is active
    /// @}
    
    /**
     * @name ROS 2 Services
     * @brief Service interfaces for robot control
     * @{
     */
    rclcpp::Service<arminator_driver::srv::MoveServo>::SharedPtr move_servo_service_;      ///< Single servo movement service
    rclcpp::Service<arminator_driver::srv::SetPosition>::SharedPtr set_position_service_;  ///< Multi-servo position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;                    ///< Emergency stop service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_service_;              ///< Reset emergency stop service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr queue_status_service_;             ///< Queue status service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_service_;                     ///< Park position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr straight_up_service_;              ///< Straight up position service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;                    ///< Ready position service
    /// @}
};