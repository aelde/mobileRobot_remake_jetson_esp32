#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
from geometry_msgs.msg import Twist

SPEED_DEFAULT = 10.0

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('speed', SPEED_DEFAULT)
        
        self.sub_L = self.create_subscription(Twist, 'L_W_Sensor_node', self.sub_L_callback, 10)
        self.sub_R = self.create_subscription(Twist, 'R_W_Sensor_node', self.sub_R_callback, 10)
        self.get_logger().warn('Controller started...')
        self.pub = self.create_publisher(Twist, 'Controller_send_value', 10)
        self.timer = self.create_timer(0.25, self.controller_callback)
        self.R_W_sen_spd = 0
        self.L_W_sen_spd = 0
        self.x_R = 0
        self.y_R = 0
    
    def sub_L_callback(self, msg):
        self.L_W_sen_spd = msg.linear.x
        
    def sub_R_callback(self, msg):
        self.R_W_sen_spd = msg.linear.x
        
    def controller_callback(self):
        # const_x_L = 10.0
        # const_y_L = 10.0
        # const_x_R = 10.0
        # const_y_R = 10.0
        # if self.x_L > const_x_L: self.get_logger().warn(f'x_L > const_x_L ... ')
        # elif self.x_L < const_x_L: self.get_logger().warn(f'x_L < const_x_L ... ')
        # else : self.get_logger().warn(f'x_L == const_x_L ... ')
        # if self.y_L > const_y_L: self.get_logger().warn(f'y_L > const_y_L ... ')
        # elif self.y_L < const_y_L: self.get_logger().warn(f'y_L < const_y_L ... ')
        # else : self.get_logger().warn(f'y_L == const_y_L ... ')
        # if self.x_R > const_x_R: self.get_logger().warn(f'x_R > const_x_R ... ')
        # elif self.x_R < const_x_R: self.get_logger().warn(f'x_R < const_x_R ... ')
        # else : self.get_logger().warn(f'x_R == const_x_R ... ')
        # if self.y_R > const_y_R: self.get_logger().warn(f'y_R > const_y_R ... ')
        # elif self.y_R < const_y_R: self.get_logger().warn(f'y_R < const_y_R ... ')
        # else : self.get_logger().warn(f'y_R == const_y_R ... ')
        
        speed_param = self.get_parameter('speed').get_parameter_value().double_value
        msg = Twist()
        msg.linear.x = speed_param
        self.pub.publish(msg)
        
       
def main(args=None):
    rclpy.init(args=args)
    t = ControllerNode()
    rclpy.spin(t)
    rclpy.shutdown()   

if __name__ == '__main__':
    main()        