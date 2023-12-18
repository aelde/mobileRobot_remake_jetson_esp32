#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
from geometry_msgs.msg import Twist
import time

STEP_DEFAULT = 1.0
PERCENT_DISPLACEMENT = 5

class RWDriver(Node):
    def __init__(self):
        super().__init__('R_W_Driver')
        self.declare_parameter('step', STEP_DEFAULT)
        self.sub = self.create_subscription(Twist, 'Controller_send_value', self.sub_callback, 10)
        
        self.pub = self.create_publisher(Twist, 'R_W_Driver_node', 10)

        self.get_logger().warn('R_W_Driver node started...')
        self.R_W_drive_spd = 0.0
        self.msg = Twist()
        
    def sub_callback(self, msg):
        step_param = self.get_parameter('step').get_parameter_value().double_value
        control_spd = msg.linear.x
        curve = control_spd * step_param / 100
        # print(f'control_spd: {control_spd} curve: {curve}')
        if self.R_W_drive_spd >=((control_spd-curve)) and self.R_W_drive_spd <= (curve + control_spd):
            self.get_logger().info(f'matchig spd: {self.R_W_drive_spd} ~ control spd: {control_spd}') 
        elif self.R_W_drive_spd < control_spd:
            self.get_logger().warn(f'incresing::spd: {self.R_W_drive_spd} to -> spd_con: {control_spd}')
            self.R_W_drive_spd += step_param
        elif self.R_W_drive_spd > control_spd:
            self.get_logger().warn(f'decresing::spd: {self.R_W_drive_spd} to -> spd_con: {control_spd}')
            self.R_W_drive_spd -= step_param
            
        self.msg.linear.x = self.R_W_drive_spd
        self.pub.publish(self.msg)
    
def main(args=None):
    rclpy.init(args=args)
    t = RWDriver()
    rclpy.spin(t)
    rclpy.shutdown()   

if __name__ == '__main__':
    main()        