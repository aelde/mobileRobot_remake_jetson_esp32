#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pappap.srv import SpeedControl

class SpeedControlClient(Node):
    def __init__(self):
        super().__init__('SpeedControlClient')
        self.cli = self.create_client(SpeedControl, 'SpeedControl')
        self.req = SpeedControl.Request()
    
    def send_request(self,num):
        self.req.spd = float(num)
        self.cli.wait_for_service()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.result = self.future.result()
        return self.result

def main(args=None):
    rclpy.init(args=args)
    s = SpeedControlClient()
    u_intput = input('Enter spd(float): ')
    r = s.send_request(u_intput)
    print(f'server return {r}')    
    s.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()