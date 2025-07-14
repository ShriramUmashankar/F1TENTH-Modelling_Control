#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from time import time

class MotorAndServoController(Node):
    def __init__(self):
        super().__init__('motor_and_servo_controller')

        # Motor and servo publishers
        self.motor_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

        # Predefined sequences
        self.duty_values = [0.0, 0.0, 0.025, 0.045, 0.05, 0.05, 0.00]
        self.servo_values = [0.5, 0.5, 0.85, 0.15, 0.15, 0.15, 0.5]  # same length as duty_values

        self.current_index = 0
        self.hold_duration = 3  # seconds per value
        self.start_time = time()

        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def timer_callback(self):
        elapsed = time() - self.start_time

        if self.current_index >= len(self.duty_values):
            self.get_logger().info("Command sequence completed. Stopping node.")
            rclpy.shutdown()
            return

        # Advance index if time has passed
        if elapsed >= self.hold_duration * (self.current_index + 1):
            self.current_index += 1
            if self.current_index >= len(self.duty_values):
                return  # Last value already published

        # Publish motor command
        motor_msg = Float64()
        motor_msg.data = self.duty_values[self.current_index]
        self.motor_pub.publish(motor_msg)

        # Publish servo command
        servo_msg = Float64()
        servo_msg.data = self.servo_values[self.current_index]
        self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorAndServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
