#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist
import time

STEP_DEFAULT = 1.0
PERCENT_DISPLACEMENT = 5


class RWSensor(Node):
    def __init__(self):
        super().__init__('R_W_Sensor')
        self.declare_parameter('step', STEP_DEFAULT)
        self.pub = self.create_publisher(Twist, 'R_W_Sensor_node', 10)        
        self.sup = self.create_subscription(Twist, 'Controller_send_value', self.sup_callback, 10)
        self.get_logger().warn('R_W_Sensor node started...')
        self.R_W_sensor_spd = 0.0
        self.msg = Twist()
        
    def sup_callback(self, msg):
        step_param = self.get_parameter('step').get_parameter_value().double_value
        controller_spd = msg.linear.x
        curve = controller_spd * step_param / 100
        
        if self.R_W_sensor_spd >=((controller_spd-curve)) and self.R_W_sensor_spd <= (curve + controller_spd):
            self.get_logger().info(f'matchig spd: {self.R_W_sensor_spd} ~ control spd: {controller_spd}') 
        elif self.R_W_sensor_spd < controller_spd:
            self.get_logger().warn(f'incresing::spd: {self.R_W_sensor_spd} to -> spd_con: {controller_spd}')
            self.R_W_sensor_spd += step_param
        elif self.R_W_sensor_spd > controller_spd:
            self.get_logger().warn(f'decresing::spd: {self.R_W_sensor_spd} to -> spd_con: {controller_spd}')
            self.R_W_sensor_spd -= step_param
        
        self.msg.linear.x = self.R_W_sensor_spd
        self.pub.publish(self.msg)
    
def main(args=None):
    rclpy.init(args=args)
    t = RWSensor()
    rclpy.spin(t)
    rclpy.shutdown()   

if __name__ == '__main__':
    main()        