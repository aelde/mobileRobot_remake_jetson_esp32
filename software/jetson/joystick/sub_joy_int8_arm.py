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
        self.json_messages = [
            {"L1": 0, "L2": 0, "R1": 0, "R2": 0}
        ]

    def update_control(self, msg):
        for i,j in enumerate(msg.data):
            if i == 2 : 
                self.arm_direction_control(j)
    def arm_direction_control(self, cmd):
        # result = 'no arm move'
        if cmd != -1 :
            arms = ['L1','L2','R1','R2']
            direc = ['up','down','right','left']
            if direc[cmd%4] == 'up' :
                self.arm_speed = 1
            elif direc[cmd%4] == 'down' :
                self.arm_speed = -1
            self.arm_cmd = f'{arms[cmd//4]}'       
        else: 
            self.arm_speed = 0
        self.armcontrol()
        
    def armcontrol(self):
        # Convert data to bytes and send it over the serial port
        L1 = self.arm_speed if self.arm_cmd == 'L1' else 0
        L2 = self.arm_speed if self.arm_cmd == 'L2' else 0
        R1 = self.arm_speed if self.arm_cmd == 'R1' else 0
        R2 = self.arm_speed if self.arm_cmd == 'R2' else 0

        print(self.json_messages)
        self.json_messages = [
            {"L1": L1, "L2": L2, "R1": R1, "R2": R2}
        ]
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