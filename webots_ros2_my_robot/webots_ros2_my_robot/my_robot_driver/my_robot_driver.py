#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from webots_ros2_driver.webots_controller import WebotsController


class MyRobotDriver(WebotsController):

    def __init__(self):
        super().__init__(robot_name='MyRobot')

        # Wielen ophalen
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')

        # Motor instellingen
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel, 10)

    def cmd_vel(self, msg):
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)


def main():
    rclpy.init()
    node = MyRobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
