#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist
import time

STEP_DEFAULT = 1.0
PERCENT_DISPLACEMENT = 5


class LWSensor(Node):
    def __init__(self):
        super().__init__('L_W_Sensor')
        self.declare_parameter('step', STEP_DEFAULT)
        self.pub = self.create_publisher(Twist, 'L_W_Sensor_node', 10)        
        self.sup = self.create_subscription(Twist, 'Controller_send_value', self.sup_callback, 10)
        self.get_logger().warn('L_W_Sensor node started...')
        self.L_W_sensor_spd = 0.0
        self.msg = Twist()
        
    def sup_callback(self, msg):
        step_param = self.get_parameter('step').get_parameter_value().double_value
        controller_spd = msg.linear.x
        curve = controller_spd * step_param / 100
        
        if self.L_W_sensor_spd >=((controller_spd-curve)) and self.L_W_sensor_spd <= (curve + controller_spd):
            self.get_logger().info(f'matchig spd: {self.L_W_sensor_spd} ~ control spd: {controller_spd}') 
        elif self.L_W_sensor_spd < controller_spd:
            self.get_logger().warn(f'incresing::spd: {self.L_W_sensor_spd} to -> spd_con: {controller_spd}')
            self.L_W_sensor_spd += step_param
        elif self.L_W_sensor_spd > controller_spd:
            self.get_logger().warn(f'decresing::spd: {self.L_W_sensor_spd} to -> spd_con: {controller_spd}')
            self.L_W_sensor_spd -= step_param
        
        self.msg.linear.x = self.L_W_sensor_spd
        self.pub.publish(self.msg)
    
def main(args=None):
    rclpy.init(args=args)
    t = LWSensor()
    rclpy.spin(t)
    rclpy.shutdown()   

if __name__ == '__main__':
    main()        