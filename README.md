# Arminator

A high-level C++ API and ROS interface for the LynxMotion AL5D robot arm (RB-Lyn-814).

## Overview

Arminator provides a comprehensive software interface that enables complex robot arm tasks by building upon the existing low-level serial protocol of the LynxMotion AL5D robot arm. The package includes:

- **High-level C++ API** for robot control
- **ROS 2 integration** with standard message types
- **Forward and inverse kinematics** solver
- **Trajectory planning** capabilities
- **Safety features** with joint limits
- **Serial communication** interface

## Features

### Core API Features
- Joint space control (position, velocity)
- Cartesian space control (x, y, z, roll, pitch, yaw)
- Trajectory execution with multiple waypoints
- Gripper control (open, close, position)
- Forward and inverse kinematics
- Safety limits and collision avoidance
- Real-time joint state feedback

### ROS 2 Integration
- Standard ROS message types (sensor_msgs, geometry_msgs)
- Services for common robot operations
- Launch files for easy deployment
- Parameter configuration
- Robot state visualization support

## Hardware Requirements

- LynxMotion AL5D Robot Arm (RB-Lyn-814)
- USB-to-Serial adapter or built-in serial port
- Linux system with ROS 2 (tested on Humble)

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> arminator
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select arminator
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Quick Start

### Basic C++ Usage

```cpp
#include "arminator/al5d_controller.hpp"

int main() {
    // Initialize controller
    arminator::AL5DController robot("/dev/ttyUSB0", 9600);
    
    if (!robot.initialize()) {
        std::cerr << "Failed to initialize robot" << std::endl;
        return -1;
    }
    
    // Home the robot
    robot.homeRobot();
    
    // Move to joint positions (degrees)
    std::vector<double> joint_positions = {45, 30, -45, 0, 0, 90};
    robot.moveToJointPositions(joint_positions, 512, true);
    
    // Move to Cartesian position (mm)
    arminator::CartesianPose target_pose(200, 100, 150, 0, 0, 45);
    robot.moveToCartesianPose(target_pose, 512, true);
    
    // Control gripper
    robot.closeGripper();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.openGripper();
    
    robot.shutdown();
    return 0;
}
```

### ROS 2 Usage

1. Launch the robot node:
```bash
ros2 launch arminator arminator.launch.py serial_port:=/dev/ttyUSB0
```

2. Control via command line:
```bash
# Home the robot
ros2 service call /home_robot std_srvs/srv/Empty

# Send joint commands
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
    '{name: ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "wrist_rotate_joint"], 
      position: [0.785, 0.5, -0.785, 0.0, 0.0]}'

# Open gripper
ros2 service call /open_gripper std_srvs/srv/Empty

# Monitor joint states
ros2 topic echo /joint_states
```

## API Reference

### AL5DController Class

#### Constructor
```cpp
AL5DController(const std::string& serial_port, int baud_rate = 9600)
```

#### Core Methods
- `bool initialize()` - Initialize connection and home robot
- `void shutdown()` - Safely shutdown robot connection
- `bool isConnected()` - Check connection status

#### Joint Control
- `bool moveToJointPositions(positions, speed, blocking)` - Move to joint angles
- `JointState getCurrentJointState()` - Get current joint positions
- `void setJointLimits(limits)` - Configure safety limits

#### Cartesian Control
- `bool moveToCartesianPose(pose, speed, blocking)` - Move to Cartesian pose
- `CartesianPose getCurrentCartesianPose()` - Get current end-effector pose

#### Trajectory Control
- `bool executeTrajectory(trajectory, blocking)` - Execute multi-point trajectory
- `void stopMotion()` - Emergency stop
- `bool isMoving()` - Check if robot is moving

#### Gripper Control
- `void openGripper(speed)` - Open gripper
- `void closeGripper(speed)` - Close gripper
- `void setGripperPosition(position, speed)` - Set specific gripper position

## Configuration

Edit `config/al5d_config.yaml` to adjust:
- Serial port settings
- Joint limits
- Servo calibration values
- Movement speeds
- Safety parameters

## ROS 2 Topics and Services

### Published Topics
- `/joint_states` (sensor_msgs/JointState) - Current joint positions
- `/cartesian_pose` (geometry_msgs/Pose) - Current end-effector pose

### Subscribed Topics
- `/joint_commands` (sensor_msgs/JointState) - Joint position commands
- `/cartesian_commands` (geometry_msgs/Pose) - Cartesian pose commands

### Services
- `/home_robot` (std_srvs/Empty) - Home robot to initial position
- `/stop_motion` (std_srvs/Empty) - Stop all motion
- `/set_torque_enable` (std_srvs/SetBool) - Enable/disable servo torque
- `/open_gripper` (std_srvs/Empty) - Open gripper
- `/close_gripper` (std_srvs/Empty) - Close gripper

## Safety Features

- Joint position limits enforcement
- Velocity limits
- Emergency stop capability
- Torque enable/disable
- Reachability checking for Cartesian targets

## Troubleshooting

### Serial Connection Issues
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify correct baud rate (default: 9600)
- Ensure no other programs are using the serial port

### Robot Not Responding
- Check power supply to robot arm
- Verify servo connections
- Test serial communication with basic commands

### Build Issues
- Ensure all ROS 2 dependencies are installed
- Check CMake version (requires 3.8+)
- Verify C++14 compiler support

## Contributing

1. Fork the repository
2. Create a feature branch
3. Implement your changes with tests
4. Submit a pull request

## License

MIT License - see LICENSE file for details.

## Hardware Specifications

The LynxMotion AL5D robot arm specifications:
- 5 degrees of freedom + gripper
- Reach: ~350mm
- Payload: ~50g
- Servo resolution: ~0.3°
- Serial communication: 9600 baud
- Power: 6V DC

## References

- [LynxMotion AL5D Documentation](https://www.lynxmotion.com/c-130-al5d.aspx)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Serial Communication Protocol](https://www.lynxmotion.com/images/html/build119.htm)
