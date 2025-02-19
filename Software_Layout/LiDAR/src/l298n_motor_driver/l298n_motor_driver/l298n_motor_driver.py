#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import Motor
from geometry_msgs.msg import Twist

class L298NMotorNode(Node):
    def __init__(self):
        super().__init__('l298n_motor_node')
        self.motor = Motor(forward=17, backward=18)  # GPIO pins
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('L298N Motor Node has been started.')

    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0:
            self.motor.forward()
        elif msg.linear.x < 0:
            self.motor.backward()
        else:
            self.motor.stop()

def main(args=None):
    rclpy.init(args=args)
    node = L298NMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()