#pragma once

#include "al5d_controller.hpp"
#include <array>

namespace arminator {

/**
 * @brief Kinematics solver for AL5D robot arm
 */
class AL5DKinematics {
public:
    /**
     * @brief Constructor with AL5D default DH parameters
     */
    AL5DKinematics();
    
    /**
     * @brief Forward kinematics: joint angles to end-effector pose
     * @param joint_angles Joint angles in degrees [base, shoulder, elbow, wrist, wrist_rotate]
     * @return End-effector pose in Cartesian coordinates
     */
    CartesianPose forwardKinematics(const std::vector<double>& joint_angles) const;
    
    /**
     * @brief Inverse kinematics: end-effector pose to joint angles
     * @param target_pose Target Cartesian pose
     * @param current_joints Current joint configuration (for selecting solution)
     * @return Joint angles in degrees, empty vector if no solution
     */
    std::vector<double> inverseKinematics(const CartesianPose& target_pose,
                                        const std::vector<double>& current_joints = {}) const;
    
    /**
     * @brief Check if target pose is reachable
     * @param pose Target Cartesian pose
     * @return true if reachable
     */
    bool isReachable(const CartesianPose& pose) const;
    
    /**
     * @brief Get workspace limits
     * @return Array of [min_x, max_x, min_y, max_y, min_z, max_z] in mm
     */
    std::array<double, 6> getWorkspaceLimits() const;
    
    /**
     * @brief Jacobian matrix for velocity control
     * @param joint_angles Current joint angles in degrees
     * @return 6x5 Jacobian matrix (6 DOF Cartesian, 5 joint DOF)
     */
    std::vector<std::vector<double>> jacobian(const std::vector<double>& joint_angles) const;
    
private:
    // AL5D DH parameters (Denavit-Hartenberg)
    struct DHParameters {
        std::vector<double> a;      // Link lengths (mm)
        std::vector<double> alpha;  // Link twists (degrees)
        std::vector<double> d;      // Link offsets (mm)
        std::vector<double> theta_offset; // Joint offsets (degrees)
    };
    
    DHParameters dh_params_;
    
    /**
     * @brief Transform matrix from DH parameters
     * @param a Link length
     * @param alpha Link twist
     * @param d Link offset
     * @param theta Joint angle
     * @return 4x4 transformation matrix
     */
    std::vector<std::vector<double>> dhTransform(double a, double alpha, double d, double theta) const;
    
    /**
     * @brief Matrix multiplication for 4x4 matrices
     */
    std::vector<std::vector<double>> matrixMultiply(
        const std::vector<std::vector<double>>& A,
        const std::vector<std::vector<double>>& B) const;
    
    /**
     * @brief Convert degrees to radians
     */
    static double degToRad(double degrees);
    
    /**
     * @brief Convert radians to degrees
     */
    static double radToDeg(double radians);
};

} // namespace arminator