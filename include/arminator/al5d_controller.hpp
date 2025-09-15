#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace arminator {

/**
 * @brief Joint position and velocity data
 */
struct JointState {
    std::vector<double> positions;  // Joint positions in degrees
    std::vector<double> velocities; // Joint velocities in degrees/sec
    std::vector<double> efforts;    // Joint efforts (not used for AL5D)
    
    JointState(size_t num_joints = 6) 
        : positions(num_joints, 0.0)
        , velocities(num_joints, 0.0)
        , efforts(num_joints, 0.0) {}
};

/**
 * @brief Robot arm limits
 */
struct JointLimits {
    std::vector<double> min_position;  // Minimum joint positions in degrees
    std::vector<double> max_position;  // Maximum joint positions in degrees
    std::vector<double> max_velocity;  // Maximum joint velocities in degrees/sec
    
    JointLimits(size_t num_joints = 6)
        : min_position(num_joints, -180.0)
        , max_position(num_joints, 180.0)
        , max_velocity(num_joints, 180.0) {}
};

/**
 * @brief Cartesian pose (position and orientation)
 */
struct CartesianPose {
    double x, y, z;           // Position in mm
    double roll, pitch, yaw;  // Orientation in degrees
    
    CartesianPose(double x = 0, double y = 0, double z = 0,
                 double roll = 0, double pitch = 0, double yaw = 0)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
};

/**
 * @brief Trajectory point for smooth motion
 */
struct TrajectoryPoint {
    JointState joint_state;
    double time_from_start;  // Time in seconds
    
    TrajectoryPoint(const JointState& state = JointState(), double time = 0.0)
        : joint_state(state), time_from_start(time) {}
};

/**
 * @brief High-level interface for LynxMotion AL5D robot arm
 */
class AL5DController {
public:
    /**
     * @brief Constructor
     * @param serial_port Serial port device path (e.g., "/dev/ttyUSB0")
     * @param baud_rate Serial communication baud rate (default: 9600)
     */
    AL5DController(const std::string& serial_port, int baud_rate = 9600);
    
    /**
     * @brief Destructor
     */
    ~AL5DController();
    
    /**
     * @brief Initialize the controller and establish connection
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Shutdown the controller and close connection
     */
    void shutdown();
    
    /**
     * @brief Check if the controller is connected and operational
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
    
    // ========== Joint Space Control ==========
    
    /**
     * @brief Move to target joint positions
     * @param positions Target joint positions in degrees [base, shoulder, elbow, wrist, wrist_rotate, gripper]
     * @param speed Movement speed (0-1023, higher is slower)
     * @param blocking Wait for completion if true
     * @return true if command sent successfully
     */
    bool moveToJointPositions(const std::vector<double>& positions, 
                             int speed = 512, bool blocking = false);
    
    /**
     * @brief Get current joint positions
     * @return Current joint state
     */
    JointState getCurrentJointState() const;
    
    /**
     * @brief Set joint limits for safety
     * @param limits Joint limits configuration
     */
    void setJointLimits(const JointLimits& limits);
    
    /**
     * @brief Get current joint limits
     * @return Current joint limits
     */
    JointLimits getJointLimits() const;
    
    // ========== Cartesian Space Control ==========
    
    /**
     * @brief Move to target Cartesian pose
     * @param pose Target pose in Cartesian coordinates
     * @param speed Movement speed (0-1023)
     * @param blocking Wait for completion if true
     * @return true if command sent successfully
     */
    bool moveToCartesianPose(const CartesianPose& pose, 
                            int speed = 512, bool blocking = false);
    
    /**
     * @brief Get current end-effector pose
     * @return Current Cartesian pose
     */
    CartesianPose getCurrentCartesianPose() const;
    
    // ========== Trajectory Control ==========
    
    /**
     * @brief Execute a trajectory of joint positions
     * @param trajectory Vector of trajectory points
     * @param blocking Wait for completion if true
     * @return true if trajectory execution started successfully
     */
    bool executeTrajectory(const std::vector<TrajectoryPoint>& trajectory, 
                          bool blocking = false);
    
    /**
     * @brief Stop current motion
     */
    void stopMotion();
    
    /**
     * @brief Check if robot is currently moving
     * @return true if moving, false if stationary
     */
    bool isMoving() const;
    
    // ========== Gripper Control ==========
    
    /**
     * @brief Open the gripper
     * @param speed Movement speed (0-1023)
     */
    void openGripper(int speed = 512);
    
    /**
     * @brief Close the gripper
     * @param speed Movement speed (0-1023)
     */
    void closeGripper(int speed = 512);
    
    /**
     * @brief Set gripper position
     * @param position Gripper position (0-180 degrees)
     * @param speed Movement speed (0-1023)
     */
    void setGripperPosition(double position, int speed = 512);
    
    // ========== Utility Functions ==========
    
    /**
     * @brief Home the robot to initial position
     * @param speed Movement speed (0-1023)
     * @return true if successful
     */
    bool homeRobot(int speed = 512);
    
    /**
     * @brief Enable/disable torque for all joints
     * @param enable true to enable torque, false to disable
     */
    void setTorqueEnable(bool enable);
    
    /**
     * @brief Set callback for joint state updates
     * @param callback Function to call when joint state changes
     */
    void setJointStateCallback(std::function<void(const JointState&)> callback);
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace arminator