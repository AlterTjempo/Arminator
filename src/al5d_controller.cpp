#include "arminator/al5d_controller.hpp"
#include "arminator/serial_interface.hpp"
#include "arminator/kinematics.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>

namespace arminator {

class AL5DController::Impl {
public:
    Impl(const std::string& serial_port, int baud_rate)
        : serial_(serial_port, baud_rate)
        , kinematics_()
        , current_joint_state_(6)
        , joint_limits_(6)
        , is_moving_(false)
        , torque_enabled_(true) {
        
        // Set default AL5D joint limits (in degrees)
        joint_limits_.min_position = {-90, -90, -90, -90, -180, 0};
        joint_limits_.max_position = {90, 90, 90, 90, 180, 180};
        joint_limits_.max_velocity = {180, 180, 180, 180, 180, 180};
        
        // Initialize current state to home position
        current_joint_state_.positions = {0, 0, 0, 0, 0, 90}; // Gripper half open
    }
    
    bool initialize() {
        if (!serial_.open()) {
            std::cerr << "Failed to open serial connection" << std::endl;
            return false;
        }
        
        // Send initialization commands
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Reset all servos to default positions
        return homeRobot(512);
    }
    
    void shutdown() {
        if (serial_.isOpen()) {
            // Stop any ongoing motion
            stopMotion();
            
            // Disable torque (relax servos)
            setTorqueEnable(false);
            
            serial_.close();
        }
    }
    
    bool isConnected() const {
        return serial_.isOpen();
    }
    
    bool moveToJointPositions(const std::vector<double>& positions, int speed, bool blocking) {
        if (!isConnected() || positions.size() < 5) {
            return false;
        }
        
        // Validate joint limits
        for (size_t i = 0; i < std::min(positions.size(), joint_limits_.min_position.size()); ++i) {
            if (positions[i] < joint_limits_.min_position[i] || 
                positions[i] > joint_limits_.max_position[i]) {
                std::cerr << "Joint " << i << " position " << positions[i] 
                         << " outside limits [" << joint_limits_.min_position[i] 
                         << ", " << joint_limits_.max_position[i] << "]" << std::endl;
                return false;
            }
        }
        
        // Convert angles to servo values and send command
        std::vector<int> servo_positions;
        for (size_t i = 0; i < positions.size() && i < 6; ++i) {
            int servo_value = SerialInterface::angleToServoValue(positions[i], i);
            servo_positions.push_back(servo_value);
        }
        
        bool success = serial_.sendMultiServoCommand(servo_positions, speed);
        
        if (success) {
            // Update current state
            for (size_t i = 0; i < positions.size() && i < current_joint_state_.positions.size(); ++i) {
                current_joint_state_.positions[i] = positions[i];
            }
            
            is_moving_ = true;
            
            // Notify callback if set
            if (joint_state_callback_) {
                joint_state_callback_(current_joint_state_);
            }
            
            if (blocking) {
                // Wait for movement to complete (estimate based on speed)
                int estimated_time_ms = 2000 + (1023 - speed) * 2; // Simple estimation
                std::this_thread::sleep_for(std::chrono::milliseconds(estimated_time_ms));
                is_moving_ = false;
            }
        }
        
        return success;
    }
    
    JointState getCurrentJointState() const {
        return current_joint_state_;
    }
    
    void setJointLimits(const JointLimits& limits) {
        joint_limits_ = limits;
    }
    
    JointLimits getJointLimits() const {
        return joint_limits_;
    }
    
    bool moveToCartesianPose(const CartesianPose& pose, int speed, bool blocking) {
        // Use inverse kinematics to convert Cartesian pose to joint angles
        std::vector<double> joint_angles = kinematics_.inverseKinematics(pose, current_joint_state_.positions);
        
        if (joint_angles.empty()) {
            std::cerr << "Target pose not reachable" << std::endl;
            return false;
        }
        
        return moveToJointPositions(joint_angles, speed, blocking);
    }
    
    CartesianPose getCurrentCartesianPose() const {
        return kinematics_.forwardKinematics(current_joint_state_.positions);
    }
    
    bool executeTrajectory(const std::vector<TrajectoryPoint>& trajectory, bool blocking) {
        if (trajectory.empty()) {
            return false;
        }
        
        // Execute trajectory points sequentially
        for (const auto& point : trajectory) {
            if (!moveToJointPositions(point.joint_state.positions, 512, true)) {
                return false;
            }
            
            // Wait for specified time
            if (point.time_from_start > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(point.time_from_start * 1000)));
            }
        }
        
        return true;
    }
    
    void stopMotion() {
        // Send stop command (disable all servos momentarily)
        for (int i = 0; i < 6; ++i) {
            serial_.sendServoCommand(i, 0, 0); // Speed 0 stops motion
        }
        is_moving_ = false;
    }
    
    bool isMoving() const {
        return is_moving_;
    }
    
    void openGripper(int speed) {
        setGripperPosition(180.0, speed); // Fully open
    }
    
    void closeGripper(int speed) {
        setGripperPosition(0.0, speed); // Fully closed
    }
    
    void setGripperPosition(double position, int speed) {
        // Gripper is joint 5
        position = std::max(0.0, std::min(180.0, position));
        
        int servo_value = SerialInterface::angleToServoValue(position, 5);
        serial_.sendServoCommand(5, servo_value, speed);
        
        current_joint_state_.positions[5] = position;
        
        if (joint_state_callback_) {
            joint_state_callback_(current_joint_state_);
        }
    }
    
    bool homeRobot(int speed) {
        // Home position: all joints at 0 degrees, gripper half open
        std::vector<double> home_positions = {0, 0, 0, 0, 0, 90};
        return moveToJointPositions(home_positions, speed, true);
    }
    
    void setTorqueEnable(bool enable) {
        torque_enabled_ = enable;
        
        if (!enable) {
            // Send commands to relax all servos
            for (int i = 0; i < 6; ++i) {
                serial_.sendServoCommand(i, 0, 0);
            }
        }
    }
    
    void setJointStateCallback(std::function<void(const JointState&)> callback) {
        joint_state_callback_ = callback;
    }
    
private:
    SerialInterface serial_;
    AL5DKinematics kinematics_;
    JointState current_joint_state_;
    JointLimits joint_limits_;
    bool is_moving_;
    bool torque_enabled_;
    std::function<void(const JointState&)> joint_state_callback_;
};

// AL5DController public interface implementation

AL5DController::AL5DController(const std::string& serial_port, int baud_rate)
    : pImpl(std::make_unique<Impl>(serial_port, baud_rate)) {}

AL5DController::~AL5DController() = default;

bool AL5DController::initialize() {
    return pImpl->initialize();
}

void AL5DController::shutdown() {
    pImpl->shutdown();
}

bool AL5DController::isConnected() const {
    return pImpl->isConnected();
}

bool AL5DController::moveToJointPositions(const std::vector<double>& positions, int speed, bool blocking) {
    return pImpl->moveToJointPositions(positions, speed, blocking);
}

JointState AL5DController::getCurrentJointState() const {
    return pImpl->getCurrentJointState();
}

void AL5DController::setJointLimits(const JointLimits& limits) {
    pImpl->setJointLimits(limits);
}

JointLimits AL5DController::getJointLimits() const {
    return pImpl->getJointLimits();
}

bool AL5DController::moveToCartesianPose(const CartesianPose& pose, int speed, bool blocking) {
    return pImpl->moveToCartesianPose(pose, speed, blocking);
}

CartesianPose AL5DController::getCurrentCartesianPose() const {
    return pImpl->getCurrentCartesianPose();
}

bool AL5DController::executeTrajectory(const std::vector<TrajectoryPoint>& trajectory, bool blocking) {
    return pImpl->executeTrajectory(trajectory, blocking);
}

void AL5DController::stopMotion() {
    pImpl->stopMotion();
}

bool AL5DController::isMoving() const {
    return pImpl->isMoving();
}

void AL5DController::openGripper(int speed) {
    pImpl->openGripper(speed);
}

void AL5DController::closeGripper(int speed) {
    pImpl->closeGripper(speed);
}

void AL5DController::setGripperPosition(double position, int speed) {
    pImpl->setGripperPosition(position, speed);
}

bool AL5DController::homeRobot(int speed) {
    return pImpl->homeRobot(speed);
}

void AL5DController::setTorqueEnable(bool enable) {
    pImpl->setTorqueEnable(enable);
}

void AL5DController::setJointStateCallback(std::function<void(const JointState&)> callback) {
    pImpl->setJointStateCallback(callback);
}

} // namespace arminator