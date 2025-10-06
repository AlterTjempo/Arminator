#! /bin/bash

# Script to run the Arminator driver node
# The ros service needs to be running before this script is executed

# Source the ROS2 workspace
source install/setup.bash

# Arm in park position
ros2 service call /park std_srvs/srv/Trigger
sleep 3 

# Move to ready position
ros2 service call /ready std_srvs/srv/Trigger
sleep 5

# Move to straight up position
ros2 service call /straight_up std_srvs/srv/Trigger
sleep 5 # Time may need be adjusted based on the speed of the arm

# Move to a different position fast.
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 0, time: 1000}"
sleep 2

# Move to a different position slow.
ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 90, time: 3000}"
sleep 4


# EStop test:
ros2 service call /ready std_srvs/srv/Trigger
sleep 1
ros2 service call /estop std_srvs/srv/Trigger
