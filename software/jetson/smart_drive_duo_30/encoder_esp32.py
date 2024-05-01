import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32, Float32 , Int32MultiArray
import json

import serial.tools.list_ports

# Constants
PPR = 330
kp = 0.4
ki = 0.001
kd = 0.08

class TestMotorControl(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
    
        self.pub_tick_FL = self.create_publisher(Int32MultiArray, 'measure_tick', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Motor speeds
        self.F_L = 0
        self.F_R = 0
        self.B_L = 0
        self.B_R = 0
        self.spd = 0
        
        # RPM tracking
        self.prevPFL = 0
        self.prevTime_FL = 0
        self.prev_err_FL = 0
        
        self.realP_FL = 0
        self.realP_FR = 0
        self.realP_RL = 0
        self.realP_RR = 0
            
        
        # Serial Ports
        # self.s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        # self.ss = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
    
    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = [self.realP_FL, self.realP_FR, self.realP_RL, self.realP_RR]
        self.pub_tick_FL.publish(msg)
        
        r = self.send_json_and_receive_response()
    
    def send_json_and_receive_response(self):
        response = self.serial_port.readline().decode('utf-8').strip()
        try:
            response_data = json.loads(response)
            print("Received response:")
            print(json.dumps(response_data, indent=2))

            # Access specific keys from the JSON object
            if 'FL_cFL' in response_data:
                self.realP_FL = response_data['FL_cFL']
                # print(f"Value for key2: {value2}")
                
            if 'FR_cFR' in response_data:
                self.realP_FR = response_data['FR_cFR']
                # print(f"Value for key1: {value1}")

            if 'RL_cRL' in response_data:
                self.realP_RL = response_data['RL_cRL']
                
            if 'RR_cRR' in response_data:
                self.realP_RR = response_data['RR_cRR']    
            
            
            # self.counterRPM(realP_FL, realP_FR, realP_RL, realP_RR)
            
            # self.computePID_FL(self.desiredRPM_FL, realP_FL)
            
            # print(f"realP_FL : {realP_FL}, realP_FR : {realP_FR}, realP_RL : {realP_RL}, realP_RR : {realP_RR}")
            
            return response_data
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON response: {e}")
            return None
        
    
        
def main(args=None):
    rclpy.init()
    server = TestMotorControl()
    print(f'motor_driver_node is running...')
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print(f'motor_driver_node is terminated!')
        server.destroy_node()

if __name__ == '__main__':
    main()
