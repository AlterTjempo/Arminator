#! /bin/bash

# Script to run the Arminator driver node
# The ros service needs to be running before this script is executed

# Source the ROS2 workspace
source install/setup.bash

# Move to ready position
ros2 service call /ready std_srvs/srv/Trigger

# Move to straight up position
ros2 service call /straight_up std_srvs/srv/Trigger

# Move to a few different positions 
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 1, angle: 0, time: 1000}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 2, angle: 90, time: 1000}"


ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 1, angle: 45, time: 1000}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 2, angle: 0, time: 1000}"

# Wave
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 0, time: 500}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 90, time: 500}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 0, time: 500}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 90, time: 500}"
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 0, time: 500}"

ros2 service call /park std_srvs/srv/Trigger
ros2 service call /straight_up std_srvs/srv/Trigger


# EStop test:
ros2 service call /ready std_srvs/srv/Trigger