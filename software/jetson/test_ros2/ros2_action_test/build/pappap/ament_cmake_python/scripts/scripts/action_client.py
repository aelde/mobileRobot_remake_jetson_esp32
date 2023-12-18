#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from pappap.action import Navigate
from geometry_msgs.msg import Point
import time
import numpy as np

class NavigateActionClient(Node):
    def __init__(self):
        super().__init__('test_action_client_node')
        self._action_client = ActionClient(self, Navigate, 'navigate')
        
    def sent_goal(self, x, y, z):
        goal_msg = Navigate.Goal()
        goal_msg.target.x = float(x)
        goal_msg.target.y = float(y)
        goal_msg.target.z = float(z) 
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback) 
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'Received Feedback distance : {feedback.distance_to_target} meters')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return None
        
        self.get_logger().info('Goal accepted...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        print(f'Result : {result.used_time} seconds')
        rclpy.shutdown()   
                
             
def main(args=None):
    rclpy.init()
    node = NavigateActionClient()
    print('Navigate action client node is running...')
    
    try:
        x = float(input('target x: '))
        y = float(input('target y: '))
        z = float(input('target z: '))
        node.sent_goal(x, y, z)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Navigate action client node has been terminated!!!')
        node.destroy_node()

if __name__ == '__main__':
    main()