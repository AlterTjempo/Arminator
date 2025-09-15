#include "arminator/al5d_controller.hpp"
#include <iostream>
#include <thread>
#include <chrono>

/**
 * @brief Example demonstrating basic AL5D robot arm control
 * 
 * This example shows how to:
 * 1. Initialize the robot arm
 * 2. Home the robot
 * 3. Move to various positions
 * 4. Perform pick and place operations
 * 5. Execute trajectories
 */

int main() {
    std::cout << "=== AL5D Robot Arm Example ===" << std::endl;
    
    // Initialize robot controller
    // Note: Change "/dev/ttyUSB0" to your actual serial port
    arminator::AL5DController robot("/dev/ttyUSB0", 9600);
    
    if (!robot.initialize()) {
        std::cerr << "Error: Failed to initialize robot. Check serial connection." << std::endl;
        return -1;
    }
    
    std::cout << "Robot initialized successfully!" << std::endl;
    
    // Set safety limits
    arminator::JointLimits limits(6);
    limits.min_position = {-90, -90, -90, -90, -180, 0};
    limits.max_position = {90, 90, 90, 90, 180, 180};
    robot.setJointLimits(limits);
    
    try {
        // 1. Home the robot
        std::cout << "\\n1. Homing robot..." << std::endl;
        robot.homeRobot(256); // Slow speed for safety
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 2. Move to various joint positions
        std::cout << "\\n2. Moving to different joint positions..." << std::endl;
        
        // Position 1: Base rotation
        std::vector<double> pos1 = {45, 0, 0, 0, 0, 90};
        robot.moveToJointPositions(pos1, 512, true);
        
        // Position 2: Reach forward
        std::vector<double> pos2 = {45, 30, -45, 15, 0, 90};
        robot.moveToJointPositions(pos2, 512, true);
        
        // Position 3: Reach up
        std::vector<double> pos3 = {0, -30, -30, -30, 0, 90};
        robot.moveToJointPositions(pos3, 512, true);
        
        // 3. Cartesian space movement
        std::cout << "\\n3. Moving in Cartesian space..." << std::endl;
        
        arminator::CartesianPose target_pose(200, 100, 150, 0, 0, 45);
        if (robot.moveToCartesianPose(target_pose, 512, true)) {
            std::cout << "Successfully moved to Cartesian position" << std::endl;
        } else {
            std::cout << "Cartesian position not reachable" << std::endl;
        }
        
        // 4. Gripper operations
        std::cout << "\\n4. Testing gripper..." << std::endl;
        
        robot.openGripper(256);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        robot.closeGripper(256);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        robot.setGripperPosition(90, 256); // Half open
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 5. Trajectory execution
        std::cout << "\\n5. Executing trajectory..." << std::endl;
        
        std::vector<arminator::TrajectoryPoint> trajectory;
        
        // Waypoint 1
        arminator::JointState state1(6);
        state1.positions = {0, 0, 0, 0, 0, 90};
        trajectory.emplace_back(state1, 0.0);
        
        // Waypoint 2
        arminator::JointState state2(6);
        state2.positions = {30, 15, -30, 0, 0, 90};
        trajectory.emplace_back(state2, 2.0);
        
        // Waypoint 3
        arminator::JointState state3(6);
        state3.positions = {-30, 15, -30, 0, 0, 90};
        trajectory.emplace_back(state3, 4.0);
        
        // Waypoint 4: Back to home
        trajectory.emplace_back(state1, 6.0);
        
        robot.executeTrajectory(trajectory, true);
        
        // 6. Pick and place example
        std::cout << "\\n6. Pick and place demonstration..." << std::endl;
        
        // Move to pickup position
        std::vector<double> pickup_pos = {30, 45, -60, 15, 0, 180}; // Open gripper
        robot.moveToJointPositions(pickup_pos, 512, true);
        
        // Close gripper to "pick"
        robot.closeGripper(256);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Lift up
        std::vector<double> lift_pos = {30, 15, -30, 0, 0, 0}; // Closed gripper
        robot.moveToJointPositions(lift_pos, 512, true);
        
        // Move to place position
        std::vector<double> place_pos = {-30, 45, -60, 15, 0, 0}; // Closed gripper
        robot.moveToJointPositions(place_pos, 512, true);
        
        // Open gripper to "place"
        robot.openGripper(256);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Return to home
        robot.homeRobot(512);
        
        std::cout << "\\n=== Example completed successfully! ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error during execution: " << e.what() << std::endl;
        robot.stopMotion();
    }
    
    // Shutdown robot
    robot.shutdown();
    std::cout << "Robot shutdown complete." << std::endl;
    
    return 0;
}