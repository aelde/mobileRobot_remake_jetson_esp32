#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

# c = 00000000 - 00111111 # bitที่ 7-8::00 , 0 - 63 ล้อซ้าย ไปข้่างหน้า
# c = 01000000 - 01111111 # bitที่ 7-8::01 , 64 - 127 ล้อซ้าย ไปข้างหลัง
# c = 10000000 - 10111111 # bitที่ 7-8::10 , 128 - 191 ล้อขวา ไปข้างหน้า
# c = 11000000 - 11111111 # bitที่ 7-8::11 , 192 - 255 ล้อขวา ไปข้างหลัง

import serial
import time
import serial.tools.list_ports
po = serial.tools.list_ports.comports()
for port in po:    
    print(port.device)

# serial_port = serial.Serial('/dev/cu.usbserial-2110', 115200, timeout=1)


class TestMotorControl(Node):
    def __init__(self):
        super().__init__('joy_control_service_server')
        self.sub = self.create_subscription(Int8MultiArray, 'joystick_topic', self.update_control, 10)
        
        # param set
        self.arm_cmd = 'no arm move'
        self.arm_speed = 0
    def update_control(self, msg):
        for i,j in enumerate(msg.data):
            if i == 2 : 
                # print(f'arm : {self.arm_direction_control(j)}')
                # self.get_logger().info(f'arm : {self.arm_direction_control(j)}')
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
            # elif direc[cmd%4] == 'right' :
            #     self.arm_speed = 3
            # elif direc[cmd%4] == 'left' :
            #     self.arm_speed = 4
            # print(f'{arms[cmd//4]} {direc[cmd%4]} speed:{self.arm_speed}')
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
        data = f'"L1":{L1},"L2":{L2},"R1":{R1},"R2":{R2} / sp:{self.arm_speed} / cmd:{self.arm_cmd}'
        # data_bytes = data.encode('utf-8')
        # serial_port.write(data_bytes)
        # print(f"Sent: {self.arm_cmd} // {data}")
        print(f"{data}")

def main(args=None):
    rclpy.init()
    server = TestMotorControl()
    print(f'JOY Control server is running...')
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print(f'driver service server terminated!')
        # serial_port.close()
        server.destroy_node()

if __name__ == '__main__':
    main()