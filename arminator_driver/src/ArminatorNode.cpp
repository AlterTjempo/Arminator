#include "arminator_driver/ArminatorNode.hpp"
#include <iostream>
#include <map>
#include <algorithm>

using namespace std::placeholders;

ArminatorNode::ArminatorNode(const ServoConfiguration &servo_config, bool test_mode)
    : Node("arminator_driver_node"), servo_config_(servo_config), driver_(servo_config.getSerialPort(), servo_config.baud_rate, test_mode),
      queue_running_(true), emergency_stop_active_(false)
{

    // Initialize current pulse widths to 1500 (center position) for all channels
    for (uint8_t channel = 0; channel < 6; ++channel)
    {
        current_pulse_widths_[channel] = 1500;
    }

    // Create services
    move_servo_service_ = this->create_service<arminator_driver::srv::MoveServo>(
        "move_servo",
        std::bind(&ArminatorNode::moveServo, this, _1, _2));

    estop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "estop",
        std::bind(&ArminatorNode::estop, this, _1, _2));

    reset_estop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_estop",
        std::bind(&ArminatorNode::resetEstop, this, _1, _2));

    queue_status_service_ = this->create_service<std_srvs::srv::Trigger>(
        "queue_status",
        std::bind(&ArminatorNode::getQueueStatus, this, _1, _2));

    park_service_ = this->create_service<std_srvs::srv::Trigger>(
        "park",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::Park));

    straight_up_service_ = this->create_service<std_srvs::srv::Trigger>(
        "straight_up",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::StraightUp));

    ready_service_ = this->create_service<std_srvs::srv::Trigger>(
        "ready",
        std::bind(&ArminatorNode::moveToPredefinedPosition, this, _1, _2, Position::Ready));

    RCLCPP_INFO(this->get_logger(), std::string(servo_config.arm_name + " is ready.").c_str());

    // Start the command queue processing thread
    queue_thread_ = std::thread(&ArminatorNode::processCommandQueue, this);

    try
    {
        auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
        moveToPredefinedPosition(nullptr, response, Position::Park);
        RCLCPP_INFO(this->get_logger(), "Setup: Moved to park position");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to park position on startup: %s", e.what());
    }
}

ArminatorNode::~ArminatorNode()
{
    // Stop the queue processing thread
    queue_running_ = false;
    if (queue_thread_.joinable())
    {
        queue_thread_.join();
    }
    driver_.sendMultiServoCommand(positions.at(Position::Park));
    RCLCPP_INFO(this->get_logger(), "Moved to park position on shutdown");
}

void ArminatorNode::moveServo(const std::shared_ptr<arminator_driver::srv::MoveServo::Request> request,
                              std::shared_ptr<arminator_driver::srv::MoveServo::Response> response)
{
    // Check if emergency stop is active
    if (emergency_stop_active_)
    {
        response->status = 1; // error
        RCLCPP_WARN(this->get_logger(), "Cannot move servo - emergency stop is active");
        return;
    }

    // Validate that both time and speed are not set
    if (request->time != 0 && request->speed != 0)
    {
        response->status = 1; // error
        RCLCPP_WARN(this->get_logger(), "Cannot set both time and speed parameters");
        return;
    }

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
    if (base_pulse_width < min_limit || base_pulse_width > max_limit)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested angle %d for servo %d results in out-of-bounds pulse width %dμs (limits: %d to %d). Command not sent.",
                    request->angle, request->servo, base_pulse_width, min_limit, max_limit);
        response->status = 1; // error
        return;
    }

    command.pulseWidth = std::max(min_limit, std::min(max_limit, base_pulse_width));

    // Set time or speed parameter
    if (request->time != 0)
    {
        command.time = request->time;
    }
    if (request->speed != 0)
    {
        command.speed = request->speed;
    }

    // Queue the command - since ROS services are synchronous, we just add to queue and return success
    queueSingleServoCommand(command, [this, request, command](bool success)
                            {
        if (success) {
            if (command.speed.has_value()) {
                RCLCPP_INFO(this->get_logger(), 
                            "Queued servo %d to angle %d (pulse width: %dμs) with speed %dμs/s", 
                            request->servo, request->angle, command.pulseWidth, command.speed.value());
            } else {
                RCLCPP_INFO(this->get_logger(), 
                            "Queued servo %d to angle %d (pulse width: %dμs) over %dms", 
                            request->servo, request->angle, command.pulseWidth, command.time.value_or(0));
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to queue servo %d command", request->servo);
        } });

    response->status = 0; // Command queued successfully
    RCLCPP_INFO(this->get_logger(), "Servo command queued for servo %d", request->servo);
}

void ArminatorNode::estop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Set emergency stop flag
    emergency_stop_active_ = true;

    // Clear the command queue
    clearCommandQueue();

    // Stop all servos immediately
    driver_.stopAllServos();

    response->success = true;
    response->message = "Emergency stop activated! Command queue cleared and all servos stopped.";

    RCLCPP_WARN(this->get_logger(), "Emergency stop activated - queue cleared and servos stopped");
}

void ArminatorNode::resetEstop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reset emergency stop flag
    emergency_stop_active_ = false;

    response->success = true;
    response->message = "Emergency stop reset - robot arm ready for commands";

    RCLCPP_INFO(this->get_logger(), "Emergency stop reset - robot arm ready for commands");
}

void ArminatorNode::getQueueStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    size_t queue_size = command_queue_.size();
    std::string emergency_status = emergency_stop_active_ ? "ACTIVE" : "INACTIVE";
    std::string queue_status = queue_running_ ? "RUNNING" : "STOPPED";

    response->success = true;
    response->message = "Queue size: " + std::to_string(queue_size) +
                        ", Emergency stop: " + emergency_status +
                        ", Queue processing: " + queue_status;

    RCLCPP_DEBUG(this->get_logger(), "Queue status requested: %s", response->message.c_str());
}

void ArminatorNode::moveToPredefinedPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request [[maybe_unused]],
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                             Position pos)
{
    // Check if emergency stop is active
    if (emergency_stop_active_)
    {
        response->success = false;
        response->message = "Cannot move to position - emergency stop is active";
        RCLCPP_WARN(this->get_logger(), "Cannot move to predefined position - emergency stop is active");
        return;
    }

    auto it = positions.find(pos);
    if (it == positions.end())
    {
        response->success = false;
        response->message = "Unknown position";
        return;
    }

    // Queue the multi-servo command
    queueMultiServoCommand(it->second, [this, pos](bool success)
                           {
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Successfully moved to predefined position");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to predefined position");
        } });

    response->success = true;
    response->message = "Position command queued successfully";
    RCLCPP_INFO(this->get_logger(), "Predefined position command queued");
}

void ArminatorNode::queueSingleServoCommand(const RobotArmDriver::ServoCommand &command,
                                            std::function<void(bool)> callback)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    SingleServoCommandInfo cmd_info;
    cmd_info.command = command;
    cmd_info.callback = callback;

    TimedCommand timed_cmd;
    timed_cmd.command = cmd_info;
    timed_cmd.execution_time = calculateExecutionTime(command);

    command_queue_.push(timed_cmd);

    RCLCPP_INFO(this->get_logger(), "Single servo command queued (servo %d, execution time: %ld ms, queue size: %zu)",
                cmd_info.command.channel, timed_cmd.execution_time.count(), command_queue_.size());
}

void ArminatorNode::queueMultiServoCommand(const RobotArmDriver::MultiServoCommand &commands,
                                           std::function<void(bool)> callback)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    MultiServoCommandInfo cmd_info;
    cmd_info.commands = commands;
    cmd_info.callback = callback;

    TimedCommand timed_cmd;
    timed_cmd.command = cmd_info;
    timed_cmd.execution_time = calculateMaxExecutionTime(commands);

    command_queue_.push(timed_cmd);

    RCLCPP_INFO(this->get_logger(), "Multi servo command queued (%zu servos, execution time: %ld ms, queue size: %zu)",
                cmd_info.commands.size(), timed_cmd.execution_time.count(), command_queue_.size());
}

void ArminatorNode::clearCommandQueue()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Clear the queue by creating a new empty queue
    std::queue<TimedCommand> empty_queue;
    command_queue_.swap(empty_queue);

    RCLCPP_INFO(this->get_logger(), "Command queue cleared");
}

void ArminatorNode::processCommandQueue()
{
    RCLCPP_INFO(this->get_logger(), "Command queue processing thread started");

    while (queue_running_)
    {
        TimedCommand current_command;
        bool has_command = false;

        // Check if there's a command to process
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!command_queue_.empty())
            {
                current_command = command_queue_.front();
                command_queue_.pop();
                has_command = true;
            }
        }

        if (has_command && !emergency_stop_active_)
        {
            // Execute the command
            RCLCPP_INFO(this->get_logger(), "Executing command from queue (execution time: %ld ms)",
                        current_command.execution_time.count());
            bool success = executeCommand(current_command);

            // If the command has a timing component, wait for it to complete
            if (current_command.execution_time.count() > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting %ld ms for command to complete",
                            current_command.execution_time.count());
                std::this_thread::sleep_for(current_command.execution_time);
                RCLCPP_INFO(this->get_logger(), "Command completed, processing next command");
            }

            // Call the callback if it exists
            std::visit([success](auto &&cmd)
                       {
                if (cmd.callback) {
                    cmd.callback(success);
                } }, current_command.command);
        }
        else
        {
            // No command available or emergency stop active, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Command queue processing thread stopped");
}

bool ArminatorNode::executeSingleServoCommand(const SingleServoCommandInfo &cmd_info)
{
    RobotArmDriver::RobotArmDriverError error = driver_.sendSingleServoCommand(cmd_info.command);

    if (error.code == RobotArmDriver::RobotArmDriverError::Code::NONE)
    {
        // Update the current pulse width for this channel
        updateCurrentPulseWidth(cmd_info.command.channel, cmd_info.command.pulseWidth);
        RCLCPP_DEBUG(this->get_logger(), "Single servo command executed successfully");
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to execute single servo command: %s", error.message.c_str());
    return false;
}

bool ArminatorNode::executeMultiServoCommand(const MultiServoCommandInfo &cmd_info)
{
    RobotArmDriver::RobotArmDriverError error = driver_.sendMultiServoCommand(cmd_info.commands);

    if (error.code == RobotArmDriver::RobotArmDriverError::Code::NONE)
    {
        // Update the current pulse widths for all channels in this command
        for (const auto &command : cmd_info.commands)
        {
            updateCurrentPulseWidth(command.channel, command.pulseWidth);
        }
        RCLCPP_DEBUG(this->get_logger(), "Multi servo command executed successfully");
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to execute multi servo command: %s", error.message.c_str());
    return false;
}

bool ArminatorNode::executeCommand(const TimedCommand &timed_command)
{
    // Check which type of command we have and execute it
    if (std::holds_alternative<SingleServoCommandInfo>(timed_command.command))
    {
        const auto &cmd_info = std::get<SingleServoCommandInfo>(timed_command.command);
        return executeSingleServoCommand(cmd_info);
    }
    else if (std::holds_alternative<MultiServoCommandInfo>(timed_command.command))
    {
        const auto &cmd_info = std::get<MultiServoCommandInfo>(timed_command.command);
        return executeMultiServoCommand(cmd_info);
    }

    return false;
}

std::chrono::milliseconds ArminatorNode::calculateMaxExecutionTime(const RobotArmDriver::MultiServoCommand &commands)
{
    std::chrono::milliseconds max_time(0);

    for (const auto &command : commands)
    {
        std::chrono::milliseconds cmd_time = calculateExecutionTime(command);
        if (cmd_time > max_time)
        {
            max_time = cmd_time;
        }
    }

    return max_time;
}

std::chrono::milliseconds ArminatorNode::calculateExecutionTime(const RobotArmDriver::ServoCommand &command)
{
    // If time is specified, use it directly
    if (command.time.has_value() && command.time.value() > 0)
    {
        return std::chrono::milliseconds(command.time.value());
    }

    // If speed is specified, calculate time based on distance and speed
    if (command.speed.has_value() && command.speed.value() > 0)
    {
        uint16_t current_pw = getCurrentPulseWidth(command.channel);
        uint16_t target_pw = command.pulseWidth;

        // Calculate the absolute distance in microseconds
        int distance = std::abs(static_cast<int>(target_pw) - static_cast<int>(current_pw));

        // Speed is in microseconds per second
        // Time = distance / speed * 1000 (to convert to milliseconds)
        double time_seconds = static_cast<double>(distance) / static_cast<double>(command.speed.value());
        int time_ms = static_cast<int>(time_seconds * 1000.0);

        RCLCPP_DEBUG(this->get_logger(),
                     "Calculated execution time for channel %d: distance=%d μs, speed=%d μs/s, time=%d ms",
                     command.channel, distance, command.speed.value(), time_ms);

        return std::chrono::milliseconds(time_ms);
    }

    // No time or speed specified, return 0 (instant movement)
    return std::chrono::milliseconds(0);
}

void ArminatorNode::updateCurrentPulseWidth(uint8_t channel, uint16_t pulseWidth)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pulse_widths_[channel] = pulseWidth;
    RCLCPP_DEBUG(this->get_logger(), "Updated channel %d pulse width to %d μs", channel, pulseWidth);
}

uint16_t ArminatorNode::getCurrentPulseWidth(uint8_t channel)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = current_pulse_widths_.find(channel);
    if (it != current_pulse_widths_.end())
    {
        return it->second;
    }
    // Default to 1500 (center position) if not yet set
    return 1500;
}