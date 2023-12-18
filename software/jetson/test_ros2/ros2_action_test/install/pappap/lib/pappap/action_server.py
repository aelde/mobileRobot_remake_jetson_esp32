#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from pappap.action import Navigate
from geometry_msgs.msg import Point
import time
import numpy as np

DISTANCE_THRESHOLD = 0.1
 
class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('test_action_server_node')
        self._action_server = ActionServer(self, Navigate, 'navigate', self.navigate_callback)
        self.sub = self.create_subscription(Point, 'robot_position_node', self.robot_position_callback, 1)
        self.robot_curret_position = None
    
    def navigate_callback(self, goal_handle):
        self.get_logger().info('Receiving goal...')
        # start_time = self.get_clock().now().to_msg().sec
        start_time =  time.time()
        robot_target_point = [ goal_handle.request.target.x ,
                               goal_handle.request.target.y ,
                               goal_handle.request.target.z ]
        print(f'robot_target_point: {robot_target_point}')
        
        while self.robot_curret_position is None:
            print('Waiting for robot current position...')
            # time.sleep(1)
            rclpy.spin_once(self, timeout_sec=1.0)
        
        dist_to_target = self.calc_dist(self.robot_curret_position, robot_target_point)
        feedback_msg = Navigate.Feedback()
        
        print('im here')
        
        while dist_to_target > DISTANCE_THRESHOLD:
            print(f'dist_to_target: {dist_to_target}')
            dist_to_target = self.calc_dist(self.robot_curret_position, robot_target_point)
            feedback_msg.distance_to_target = dist_to_target 
            goal_handle.publish_feedback(feedback_msg)
            # time.sleep(1)
            rclpy.spin_once(self, timeout_sec=1.0)
        
        print('im here 2')
        goal_handle.succeed()
        result = Navigate.Result()
        result.used_time = float(time.time() - start_time)
        return result
            
    def robot_position_callback(self, msg):
        self.robot_curret_position = [ msg.x , msg.y , msg.z ]
    
    def calc_dist(self, current_points, target_points): # ปีทากอรัส
        if current_points[0] == target_points[0]:
            
            print('x is equal')
            y_len = abs(current_points[1] - target_points[1])
            print(f'y_len: {y_len}')
            
            dist = float(y_len)
        elif current_points[1] == target_points[1]:
            print('y is equal')
            x_len = abs(current_points[0] - target_points[0])
            print(f'x_len: {x_len}')
            dist = float(x_len)
        else:
            x_len = abs(current_points[0] - target_points[0])
            y_len = abs(current_points[1] - target_points[1])
            z_len = abs(current_points[2] - target_points[2])
            dist = float(np.sqrt(x_len**2 + y_len**2))
        # print(f'time: {self.get_clock().now().to_msg().nanosec}')
        return dist
                
             
def main(args=None):
    rclpy.init()
    node = NavigateActionServer()
    print('Navigate action server node is running...')
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('Navigate action server node has been terminated!!!')
        node._action_server.destroy()
        node.destroy_node()

if __name__ == '__main__':
    main()