#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from webots_ros2_driver.webots_controller import WebotsController

class MyRobotDriver(WebotsController):
    def __init__(self, node):
        super().__init__(node)

        # Alle 4 de motoren ophalen uit de PROTO
        self.left_motors = [
            self.robot.getDevice('left wheel motor'),
            self.robot.getDevice('left wheel motor 2')
        ]
        self.right_motors = [
            self.robot.getDevice('right wheel motor'),
            self.robot.getDevice('right wheel motor 2')
        ]

        # Alle motoren instellen op velocity control
        for motor in self.left_motors + self.right_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z
        
        for motor in self.left_motors:
            motor.setVelocity(left_speed)
        for motor in self.right_motors:
            motor.setVelocity(right_speed)

def main(args=None):
    rclpy.init(args=args)
    # Webots_ros2_driver handelt de node creatie meestal af via de plugin interface
    # Dit script wordt vaak aangeroepen via een ROS 2 launch file