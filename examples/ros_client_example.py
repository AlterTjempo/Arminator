#!/usr/bin/env python3
"""
Example ROS 2 client for the Arminator AL5D robot arm

This script demonstrates how to interact with the robot arm using ROS 2
topics and services.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, SetBool
import time

class ArminatorClient(Node):
    def __init__(self):
        super().__init__('arminator_client')
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointState, 'joint_commands', 10)
        self.cartesian_command_pub = self.create_publisher(
            Pose, 'cartesian_commands', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Service clients
        self.home_client = self.create_client(Empty, 'home_robot')
        self.stop_client = self.create_client(Empty, 'stop_motion')
        self.torque_client = self.create_client(SetBool, 'set_torque_enable')
        self.open_gripper_client = self.create_client(Empty, 'open_gripper')
        self.close_gripper_client = self.create_client(Empty, 'close_gripper')
        
        self.current_joint_state = None
        
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.get_logger().info(f'Joint positions: {[f"{pos:.2f}" for pos in msg.position]}')
    
    def home_robot(self):
        """Home the robot to initial position"""
        req = Empty.Request()
        future = self.home_client.call_async(req)
        return future
    
    def stop_motion(self):
        """Stop all robot motion"""
        req = Empty.Request()
        future = self.stop_client.call_async(req)
        return future
    
    def set_torque_enable(self, enable):
        """Enable or disable servo torque"""
        req = SetBool.Request()
        req.data = enable
        future = self.torque_client.call_async(req)
        return future
    
    def open_gripper(self):
        """Open the gripper"""
        req = Empty.Request()
        future = self.open_gripper_client.call_async(req)
        return future
    
    def close_gripper(self):
        """Close the gripper"""
        req = Empty.Request()
        future = self.close_gripper_client.call_async(req)
        return future
    
    def move_to_joint_positions(self, positions):
        """Move robot to specified joint positions (in radians)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["base_joint", "shoulder_joint", "elbow_joint", 
                   "wrist_joint", "wrist_rotate_joint"]
        msg.position = positions
        self.joint_command_pub.publish(msg)
    
    def move_to_cartesian_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Move robot to specified Cartesian pose"""
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_command_pub.publish(msg)

def main():
    rclpy.init()
    
    client = ArminatorClient()
    
    # Wait for services to be available
    client.get_logger().info("Waiting for services...")
    client.home_client.wait_for_service()
    client.get_logger().info("Services available!")
    
    try:
        # Home the robot
        client.get_logger().info("Homing robot...")
        future = client.home_robot()
        rclpy.spin_until_future_complete(client, future)
        time.sleep(3)
        
        # Move to different positions
        client.get_logger().info("Moving to position 1...")
        client.move_to_joint_positions([0.785, 0.0, 0.0, 0.0, 0.0])  # 45 degrees base
        time.sleep(2)
        
        client.get_logger().info("Moving to position 2...")
        client.move_to_joint_positions([0.785, 0.524, -0.785, 0.262, 0.0])  # Complex pose
        time.sleep(2)
        
        # Test Cartesian movement
        client.get_logger().info("Moving in Cartesian space...")
        client.move_to_cartesian_pose(0.2, 0.1, 0.15)  # 200mm, 100mm, 150mm
        time.sleep(2)
        
        # Test gripper
        client.get_logger().info("Testing gripper...")
        future = client.open_gripper()
        rclpy.spin_until_future_complete(client, future)
        time.sleep(1)
        
        future = client.close_gripper()
        rclpy.spin_until_future_complete(client, future)
        time.sleep(1)
        
        # Return home
        client.get_logger().info("Returning home...")
        future = client.home_robot()
        rclpy.spin_until_future_complete(client, future)
        
        client.get_logger().info("Demo completed successfully!")
        
    except KeyboardInterrupt:
        client.get_logger().info("Demo interrupted by user")
        future = client.stop_motion()
        rclpy.spin_until_future_complete(client, future)
    
    except Exception as e:
        client.get_logger().error(f"Error during demo: {e}")
        future = client.stop_motion()
        rclpy.spin_until_future_complete(client, future)
    
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()