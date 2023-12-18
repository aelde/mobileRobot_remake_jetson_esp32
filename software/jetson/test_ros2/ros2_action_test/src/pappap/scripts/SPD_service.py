#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pappap.srv import SpeedControl
from geometry_msgs.msg import Twist

class SpeedService(Node):
    def __init__(self):
        super().__init__('SpeedService')
        self.srv = self.create_service(SpeedControl, 'SpeedControl', self.srv_callback)
        self.pub = self.create_publisher(Twist, 'Controller_send_value', 10)
        self.get_logger().info('SpeedService started.....')
    
    def srv_callback(self, req, res):
        pass