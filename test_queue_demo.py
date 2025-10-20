#!/usr/bin/env python3
"""
Demo script to test the command queueing functionality of the Arminator robot arm.
This script demonstrates how multiple commands are queued and executed sequentially.
"""

import rclpy
from rclpy.node import Node
from arminator_driver.srv import MoveServo
from std_srvs.srv import Trigger
import time

class ArminatorQueueDemo(Node):
    def __init__(self):
        super().__init__('arminator_queue_demo')
        
        # Create service clients
        self.move_servo_client = self.create_client(MoveServo, 'move_servo')
        self.estop_client = self.create_client(Trigger, 'estop')
        self.reset_estop_client = self.create_client(Trigger, 'reset_estop')
        self.queue_status_client = self.create_client(Trigger, 'queue_status')
        self.park_client = self.create_client(Trigger, 'park')
        self.ready_client = self.create_client(Trigger, 'ready')
        
        # Wait for services to be available
        self.get_logger().info("Waiting for services...")
        self.move_servo_client.wait_for_service(timeout_sec=10.0)
        self.estop_client.wait_for_service(timeout_sec=10.0)
        self.reset_estop_client.wait_for_service(timeout_sec=10.0)
        self.queue_status_client.wait_for_service(timeout_sec=10.0)
        self.park_client.wait_for_service(timeout_sec=10.0)
        self.ready_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("All services are available!")
        
    def get_queue_status(self):
        """Get and print the current queue status."""
        request = Trigger.Request()
        future = self.queue_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Queue Status: {response.message}")
        else:
            self.get_logger().error("Failed to get queue status")
    
    def move_servo(self, servo_id, angle, time_ms=0):
        """Move a servo to a specific angle."""
        request = MoveServo.Request()
        request.servo = servo_id
        request.angle = angle
        request.time = time_ms
        
        future = self.move_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.status == 0:
                self.get_logger().info(f"Servo {servo_id} move command queued successfully (angle: {angle}, time: {time_ms}ms)")
            else:
                self.get_logger().error(f"Failed to queue servo {servo_id} move command")
        else:
            self.get_logger().error("Service call failed")
    
    def emergency_stop(self):
        """Trigger emergency stop."""
        request = Trigger.Request()
        future = self.estop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Emergency Stop: {response.message}")
        else:
            self.get_logger().error("Emergency stop service call failed")
    
    def reset_emergency_stop(self):
        """Reset emergency stop."""
        request = Trigger.Request()
        future = self.reset_estop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Reset Emergency Stop: {response.message}")
        else:
            self.get_logger().error("Reset emergency stop service call failed")
    
    def move_to_park(self):
        """Move to park position."""
        request = Trigger.Request()
        future = self.park_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Park: {response.message}")
        else:
            self.get_logger().error("Park service call failed")
    
    def move_to_ready(self):
        """Move to ready position."""
        request = Trigger.Request()
        future = self.ready_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Ready: {response.message}")
        else:
            self.get_logger().error("Ready service call failed")

def main():
    rclpy.init()
    demo = ArminatorQueueDemo()
    
    try:
        # Demo 1: Queue multiple commands with different timing
        demo.get_logger().info("=== DEMO 1: Queueing multiple timed commands ===")
        demo.get_queue_status()
        
        # Queue several commands with different execution times
        demo.move_servo(1, 45, 2000)  # 2 seconds
        demo.move_servo(2, 90, 1500)  # 1.5 seconds
        demo.move_servo(3, -30, 3000) # 3 seconds (longest)
        demo.move_servo(4, 60, 1000)  # 1 second
        
        demo.get_queue_status()
        demo.get_logger().info("Commands queued. The queue will process them sequentially...")
        demo.get_logger().info("Each command will wait for its specified time before the next executes.")
        
        # Wait for commands to process
        time.sleep(12)  # Wait for all commands to complete (2+1.5+3+1 = 7.5s + some buffer)
        demo.get_queue_status()
        
        # Demo 2: Queue predefined position commands
        demo.get_logger().info("\n=== DEMO 2: Queueing predefined position commands ===")
        demo.move_to_ready()
        time.sleep(2)
        demo.move_to_park()
        time.sleep(2)
        demo.get_queue_status()
        
        # Demo 3: Emergency stop functionality
        demo.get_logger().info("\n=== DEMO 3: Emergency stop functionality ===")
        # Queue some commands
        demo.move_servo(1, 0, 5000)   # 5 second command
        demo.move_servo(2, 45, 3000)  # 3 second command
        demo.get_queue_status()
        
        # Trigger emergency stop after a short delay
        time.sleep(1)
        demo.get_logger().info("Triggering emergency stop - this should clear the queue!")
        demo.emergency_stop()
        demo.get_queue_status()
        
        # Try to queue a command while emergency stop is active
        demo.get_logger().info("Trying to queue command while emergency stop is active...")
        demo.move_servo(1, 90, 1000)
        
        # Reset emergency stop
        time.sleep(2)
        demo.get_logger().info("Resetting emergency stop...")
        demo.reset_emergency_stop()
        demo.get_queue_status()
        
        # Queue a command after reset
        demo.get_logger().info("Queueing command after emergency stop reset...")
        demo.move_servo(1, 45, 2000)
        demo.get_queue_status()
        
        # Wait for final command to complete
        time.sleep(3)
        demo.get_queue_status()
        
        demo.get_logger().info("=== Demo completed successfully! ===")
        
    except Exception as e:
        demo.get_logger().error(f"Demo failed: {str(e)}")
    
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()