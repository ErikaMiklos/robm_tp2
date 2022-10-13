#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from robm_nxt_msgs.msg import MotorCommand
from geometry_msgs.msg import Twist
from math import pi

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller')
        # Publisher for NXT motor commands
        self.pub = self.create_publisher(MotorCommand, "nxt/command", 1)
        # Subscribe to motor rotation status
        self.motor_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
    
    def cmd_vel_callback(self, cmd_vel_msg:Twist):
        # Extract speed command from msg
        v = cmd_vel_msg.linear.x
        w = cmd_vel_msg.angular.z
        
        k = 16.8      # motor speed factor (rad/s)
        r = 0.0166    # wheel radius
        l = 0.15/2.0  # half track

        # Linear wheel speeds
        v1 = v + l * w
        v2 = v - l * w

        # Motor rotation speed commands
        u1 = v1 / (r * k)
        u2 = v2 / (r * k)

        # Limit motor speed to +/-1.0
        u_max = max( abs(u1), abs(u2) )
        if u_max > 1.0:
            u1 = u1 / u_max
            u2 = u2 / u_max
        
        # Publish odometry message
        msg = MotorCommand(speed_b=u1, speed_c=u2)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
