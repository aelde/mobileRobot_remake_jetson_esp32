#!/usr/bin/env python3 
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

import serial
import time
import serial.tools.list_ports
po = serial.tools.list_ports.comports()
for port in po:    
    print(port.device)

serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

class TestMotorControl(Node):
    def __init__(self):
        super().__init__('joy_control_service_server')
        self.sub = self.create_subscription(Int8MultiArray, 'joystick_topic', self.update_control, 10)
        
        # param set
        self.arm_cmd = 'no arm move'
        self.arm_speed = 0
        self.toggle_OE = 1
        self.json_messages = [
            # {"L1_a6": 0, "L2_a5": 0, "R1_a4": 0, "L1_a3": 0, "L2_a2": 0, "R2_a1": 0, "OE": 1} 
            { "R2_a1": 0, "L2_a2": 0, "L1_a3": 0, "R1_a4": 0, "L2_a5": 0, "L1_a6":0,"OE":1}
        ]
    def update_control(self, msg):
        for i,j in enumerate(msg.data):
            if i == 2 : self.arm_direction_control(j)
            if i == 3 : self.toggle_OE = j
    def arm_direction_control(self, cmd):
        # result = 'no arm move'
        if cmd != -1 :
            arms = ['L1','L2','R1','R2']
            direc = ['up','down','right','left']
            if direc[cmd%4] == 'up' or direc[cmd%4] == 'right' :
                self.arm_speed = 1
            elif direc[cmd%4] == 'down' or direc[cmd%4] == 'left' :
                self.arm_speed = -1
            self.arm_cmd = f'{arms[cmd//4]}_{direc[cmd%4]}'       
        else: 
            self.arm_speed = 0
        self.armcontrol()
        
    def armcontrol(self):
        # Convert data to bytes and send it over the serial port
        L1_a6 = self.arm_speed if self.arm_cmd == 'L1_up' or self.arm_cmd == 'L1_down' else 0
        L2_a5 = self.arm_speed if self.arm_cmd == 'L2_up' or self.arm_cmd == 'L2_down' else 0
        L1_2_a3 = self.arm_speed if self.arm_cmd == 'L1_right' or self.arm_cmd == 'L1_left' else 0
        L2_2_a2 = self.arm_speed if self.arm_cmd == 'L2_right' or self.arm_cmd == 'L2_left' else 0
        R1_a4 = self.arm_speed if self.arm_cmd == 'R1_up' or self.arm_cmd == 'R1_down' else 0
        R2_a1 = self.arm_speed if self.arm_cmd == 'R2_up' or self.arm_cmd == 'R2_down' else 0
        
        self.json_messages = [
            # {"L1_a6": L1_a6, "L2_a5": L2_a5, "R1_a4": R1_a4, "L1_a3": L1_2_a3, "L2_a2": L2_2_a2, "R2_a1": R2_a1, "OE": self.toggle_OE} 
             { "R2_a1": R2_a1, "L2_a2": L2_2_a2, "L1_a3": L1_2_a3, "R1_a4": R1_a4, "L2_a5": L2_a5, "L1_a6": L1_a6, "OE": self.toggle_OE}
            # {"L1_a6": 0, "L2_a5": 0, "L1_a3": 0, "L2_a2": 0, "R1_a4": 0, "R2_a1": 0, "OE": 1} 
        ]
        
        print(self.json_messages)
        self.json_send(self.json_messages)
        
    def json_send(self,json_messages):
        for json_message in json_messages:
        # Send JSON message and receive response
            response = self.send_json_and_receive_response(json_message)
        if response:
            print("Received response:")
            print(json.dumps(response, indent=2))
        else:
            print("No valid response received")
            
    def send_json_and_receive_response(self,json_data):
        # Send JSON data to ESP32
        formatted_json = json.dumps(json_data)
        serial_port.write((formatted_json + '\n').encode('utf-8'))

        # Read response from ESP32
        response = serial_port.readline().decode('utf-8').strip()

        try:
            # Parse the received JSON response
            response_data = json.loads(response)
            return response_data
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON response: {e}")
            return None

def main(args=None):
    rclpy.init()
    server = TestMotorControl()
    print(f'JOY Control server is running...')
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print(f'driver service server terminated!')
        serial_port.close()
        server.destroy_node()

if __name__ == '__main__':
    main()