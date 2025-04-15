#!/usr/bin/env python3
'''Node specification for teleoperation of the robot'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import me416_utilities as mu
import robot_model as rm

class MotorCommand(Node):
    '''
    A node for teleoperation of the robot
    '''
    def __init__(self):
        super().__init__('motor_command')
        self.motor_left = mu.MotorSpeedLeft()
        self.motor_right = mu.MotorSpeedRight()
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Twist, 'motor_speeds', 10)

    def cmd_vel_callback(self, msg):
        """ callback for cmd_vel """
        # pull linear and angular speeds from Twist msg
        speed_linear = msg.linear.x
        speed_angular = msg.angular.z
        # convert to left and right speeds
        speed_left, speed_right = rm.twist_to_speeds(speed_linear, speed_angular)
        # set acquired speeds to motor
        self.motor_left.set_speed(speed_left)
        self.motor_right.set_speed(speed_right)

        self.get_logger().info(f'Left Motor Speed: {speed_left}, Right Motor Speed: {speed_right}')
def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    minimal_subscriber = MotorCommand()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly,
    # otherwise it will be done automatically
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
