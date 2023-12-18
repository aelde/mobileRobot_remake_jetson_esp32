#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from pappap.action import Navigate
import time

class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')
        self.action_server = ActionServer(
            self,
            Navigate,
            'navigate_action',
            self.action_exec_callback
        )
        self.get_logger().info('Navigate action serv started...')
        self.x = 0
        self.y = 0
        self.z = 0
        self.t = 0
    
    def action_exec_callback(self, goal_handle):
        action_t = 0
        self.get_logger().info(f'Received request!')
        target_point = [goal_handle.request.target.x,
                        goal_handle.request.target.y,
                        goal_handle.request.target.z]
        self.get_logger().info(f'Target point: {target_point}')
        while self.x != target_point[0] or self.y != target_point[1] or self.z != target_point[2]:
            if target_point[0] != 0 and target_point[0] > self.x:
                self.x += 1
            elif target_point[0] != 0 and target_point[0] < self.x:
                self.x -= 1
            elif target_point[0] == 0: pass
            if target_point[1] != 0 and target_point[1] > self.y:
                self.y += 1
            elif target_point[1] != 0 and target_point[1] < self.y:
                self.y -= 1
            elif target_point[1] == 0: pass
            time.sleep(1)
            action_t += 1
            self.t += 1
            self.get_logger().warn(f'current pos : {self.x}, {self.y}, {self.z}')
        goal_handle.succeed()
        tt = Navigate.Result()
        tt.status = 'total time moving ' + str(self.t) + ' secs' + 'action moving ' + str(action_t) + ' secs'
        goal_handle.publish_feedback(tt) 
        return tt

def main(args=None):
    rclpy.init(args=args)
    a = NavigateActionServer()
    rclpy.spin(a)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        
    