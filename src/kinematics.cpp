#include "arminator/kinematics.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace arminator {

AL5DKinematics::AL5DKinematics() {
    // AL5D Denavit-Hartenberg parameters (approximate values - adjust based on actual robot)
    // Joint order: base, shoulder, elbow, wrist, wrist_rotate
    dh_params_.a = {0, 146.05, 187.325, 0, 0};           // Link lengths in mm
    dh_params_.alpha = {90, 0, 0, 90, 0};                // Link twists in degrees
    dh_params_.d = {169.77, 0, 0, 0, 101.6};            // Link offsets in mm
    dh_params_.theta_offset = {0, 90, 0, 0, 0};         // Joint offsets in degrees
}

CartesianPose AL5DKinematics::forwardKinematics(const std::vector<double>& joint_angles) const {
    if (joint_angles.size() < 5) {
        return CartesianPose();
    }
    
    // Start with identity matrix
    std::vector<std::vector<double>> T = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    
    // Multiply transformation matrices for each joint
    for (size_t i = 0; i < 5; ++i) {
        double theta = degToRad(joint_angles[i] + dh_params_.theta_offset[i]);
        std::vector<std::vector<double>> Ti = dhTransform(
            dh_params_.a[i],
            degToRad(dh_params_.alpha[i]),
            dh_params_.d[i],
            theta
        );
        T = matrixMultiply(T, Ti);
    }
    
    // Extract position and orientation from transformation matrix
    CartesianPose pose;
    pose.x = T[0][3];
    pose.y = T[1][3];
    pose.z = T[2][3];
    
    // Extract Euler angles (ZYX convention)
    pose.yaw = radToDeg(atan2(T[1][0], T[0][0]));
    pose.pitch = radToDeg(atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2])));
    pose.roll = radToDeg(atan2(T[2][1], T[2][2]));
    
    return pose;
}

std::vector<double> AL5DKinematics::inverseKinematics(const CartesianPose& target_pose,
                                                     const std::vector<double>& current_joints) const {
    // Simplified analytical inverse kinematics for AL5D
    // This is a basic implementation - more sophisticated methods may be needed for accuracy
    
    std::vector<double> joint_angles(5, 0.0);
    
    // Base rotation (joint 0)
    joint_angles[0] = radToDeg(atan2(target_pose.y, target_pose.x));
    
    // Distance from base to wrist center
    double r = sqrt(target_pose.x * target_pose.x + target_pose.y * target_pose.y);
    double wrist_z = target_pose.z - dh_params_.d[0]; // Subtract base height
    
    // Account for wrist length (assuming wrist is perpendicular to forearm)
    double wrist_length = dh_params_.d[4]; // End effector length
    double wrist_x = r - wrist_length * cos(degToRad(target_pose.pitch));
    wrist_z = wrist_z - wrist_length * sin(degToRad(target_pose.pitch));
    
    // Distance to wrist center
    double dist_to_wrist = sqrt(wrist_x * wrist_x + wrist_z * wrist_z);
    
    // Check if target is reachable
    double upper_arm = dh_params_.a[1];  // Shoulder to elbow
    double forearm = dh_params_.a[2];    // Elbow to wrist
    
    if (dist_to_wrist > (upper_arm + forearm) || dist_to_wrist < fabs(upper_arm - forearm)) {
        // Target not reachable
        return {};
    }
    
    // Shoulder angle (joint 1)
    double alpha = atan2(wrist_z, wrist_x);
    double beta = acos((upper_arm*upper_arm + dist_to_wrist*dist_to_wrist - forearm*forearm) / 
                      (2 * upper_arm * dist_to_wrist));
    joint_angles[1] = radToDeg(alpha + beta) - dh_params_.theta_offset[1];
    
    // Elbow angle (joint 2)
    double gamma = acos((upper_arm*upper_arm + forearm*forearm - dist_to_wrist*dist_to_wrist) / 
                       (2 * upper_arm * forearm));
    joint_angles[2] = 180.0 - radToDeg(gamma);
    
    // Wrist angle (joint 3) - maintain desired pitch
    joint_angles[3] = target_pose.pitch - joint_angles[1] - joint_angles[2];
    
    // Wrist rotation (joint 4)
    joint_angles[4] = target_pose.yaw;
    
    return joint_angles;
}

bool AL5DKinematics::isReachable(const CartesianPose& pose) const {
    auto joint_solution = inverseKinematics(pose);
    return !joint_solution.empty();
}

std::array<double, 6> AL5DKinematics::getWorkspaceLimits() const {
    // Approximate workspace limits for AL5D (in mm)
    // These values should be refined based on actual robot measurements
    double upper_arm = dh_params_.a[1];
    double forearm = dh_params_.a[2];
    double base_height = dh_params_.d[0];
    double max_reach = upper_arm + forearm + dh_params_.d[4];
    double min_reach = 50.0; // Minimum practical reach
    
    return {
        -max_reach,  // min_x
        max_reach,   // max_x
        -max_reach,  // min_y
        max_reach,   // max_y
        0.0,         // min_z (above base)
        base_height + max_reach  // max_z
    };
}

std::vector<std::vector<double>> AL5DKinematics::jacobian(const std::vector<double>& joint_angles) const {
    // 6x5 Jacobian matrix (6 DOF Cartesian, 5 joint DOF)
    std::vector<std::vector<double>> J(6, std::vector<double>(5, 0.0));
    
    // Numerical differentiation to compute Jacobian
    const double delta = 0.01; // Small angle change in degrees
    
    CartesianPose current_pose = forwardKinematics(joint_angles);
    
    for (int i = 0; i < 5; ++i) {
        std::vector<double> joint_plus = joint_angles;
        joint_plus[i] += delta;
        
        CartesianPose pose_plus = forwardKinematics(joint_plus);
        
        // Partial derivatives
        J[0][i] = (pose_plus.x - current_pose.x) / delta;      // dx/dqi
        J[1][i] = (pose_plus.y - current_pose.y) / delta;      // dy/dqi
        J[2][i] = (pose_plus.z - current_pose.z) / delta;      // dz/dqi
        J[3][i] = (pose_plus.roll - current_pose.roll) / delta;   // droll/dqi
        J[4][i] = (pose_plus.pitch - current_pose.pitch) / delta; // dpitch/dqi
        J[5][i] = (pose_plus.yaw - current_pose.yaw) / delta;     // dyaw/dqi
    }
    
    return J;
}

std::vector<std::vector<double>> AL5DKinematics::dhTransform(double a, double alpha, double d, double theta) const {
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    return {
        {ct, -st*ca, st*sa, a*ct},
        {st, ct*ca, -ct*sa, a*st},
        {0, sa, ca, d},
        {0, 0, 0, 1}
    };
}

std::vector<std::vector<double>> AL5DKinematics::matrixMultiply(
    const std::vector<std::vector<double>>& A,
    const std::vector<std::vector<double>>& B) const {
    
    size_t rows_A = A.size();
    size_t cols_A = A[0].size();
    size_t cols_B = B[0].size();
    
    std::vector<std::vector<double>> C(rows_A, std::vector<double>(cols_B, 0.0));
    
    for (size_t i = 0; i < rows_A; ++i) {
        for (size_t j = 0; j < cols_B; ++j) {
            for (size_t k = 0; k < cols_A; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    
    return C;
}

double AL5DKinematics::degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

double AL5DKinematics::radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

} // namespace arminator