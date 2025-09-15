#include "arminator/al5d_controller.hpp"
#include "arminator/kinematics.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

int main() {
    std::cout << "=== Arminator AL5D Controller Test ===" << std::endl;
    
    // Test basic data structures
    std::cout << "Testing data structures..." << std::endl;
    
    arminator::JointState joint_state(6);
    joint_state.positions = {0, 30, -45, 15, 0, 90};
    std::cout << "✓ JointState created with " << joint_state.positions.size() << " joints" << std::endl;
    
    arminator::CartesianPose pose(200, 100, 150, 0, 15, 30);
    std::cout << "✓ CartesianPose created: x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z << std::endl;
    
    arminator::JointLimits limits(6);
    limits.min_position = {-90, -90, -90, -90, -180, 0};
    limits.max_position = {90, 90, 90, 90, 180, 180};
    std::cout << "✓ JointLimits configured" << std::endl;
    
    // Test kinematics
    std::cout << "\\nTesting kinematics..." << std::endl;
    
    arminator::AL5DKinematics kinematics;
    
    // Test forward kinematics
    std::vector<double> test_joints = {0, 0, 0, 0, 0};
    auto forward_pose = kinematics.forwardKinematics(test_joints);
    std::cout << "✓ Forward kinematics: " << forward_pose.x << ", " << forward_pose.y << ", " << forward_pose.z << std::endl;
    
    // Test inverse kinematics
    arminator::CartesianPose target(200, 0, 200, 0, 0, 0);
    auto inverse_joints = kinematics.inverseKinematics(target);
    if (!inverse_joints.empty()) {
        std::cout << "✓ Inverse kinematics solution found" << std::endl;
    } else {
        std::cout << "⚠ Inverse kinematics: no solution found" << std::endl;
    }
    
    // Test workspace limits
    auto workspace = kinematics.getWorkspaceLimits();
    std::cout << "✓ Workspace limits: x[" << workspace[0] << "," << workspace[1] 
              << "] y[" << workspace[2] << "," << workspace[3] 
              << "] z[" << workspace[4] << "," << workspace[5] << "]" << std::endl;
    
    // Test reachability
    bool reachable = kinematics.isReachable(target);
    std::cout << "✓ Target reachability: " << (reachable ? "reachable" : "not reachable") << std::endl;
    
    // Test controller creation (won't connect without hardware)
    std::cout << "\\nTesting controller creation..." << std::endl;
    
    arminator::AL5DController controller("/dev/null", 9600);  // Use /dev/null for testing
    std::cout << "✓ Controller created" << std::endl;
    
    // Test joint limits setting
    controller.setJointLimits(limits);
    auto retrieved_limits = controller.getJointLimits();
    std::cout << "✓ Joint limits set and retrieved" << std::endl;
    
    // Test current state
    auto current_state = controller.getCurrentJointState();
    std::cout << "✓ Current joint state retrieved" << std::endl;
    
    // Test trajectory point creation
    arminator::TrajectoryPoint traj_point(joint_state, 2.5);
    std::cout << "✓ Trajectory point created with time: " << traj_point.time_from_start << "s" << std::endl;
    
    std::cout << "\\n=== All Tests Completed Successfully ===" << std::endl;
    std::cout << "Note: Hardware connection tests skipped (no serial device)" << std::endl;
    
    return 0;
}