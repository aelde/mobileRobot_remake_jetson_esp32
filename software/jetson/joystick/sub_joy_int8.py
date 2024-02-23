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

s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ss = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

class TestMotorControl(Node):
    def __init__(self):
        super().__init__('joy_control_service_server')
        self.sub = self.create_subscription(Int8MultiArray, 'joystick_topic', self.update_control, 10)
        
        # param set
        self.spd = 0 # max 63
        self.F_L = 0
        self.F_R = 0
        self.B_L = 0
        self.B_R = 0
    def update_control(self, msg):
        for i,j in enumerate(msg.data):
            if i == 0 : 
                self.spd = j
                # print(f'spd : {self.spd}')
                self.get_logger().info(f'spd : {self.spd}')
            if i == 1 : 
                # print(f'car : {self.car_direction_control(j)}')
                self.get_logger().info(f'car : {self.car_direction_control(j)}')
            if i == 2 : 
                # print(f'arm : {self.arm_direction_control(j)}')
                self.get_logger().info(f'arm : {self.arm_direction_control(j)}')
# mode   [stop a, stop, for, back, left, right, for left, for right, bac left, bac right, t left, l right]
# nuber  [-1    , 0   , 1  , 2   , 3   , 4    , 5       , 6        , 7       , 8        , 9     , 10]
    def car_direction_control(self, cmd):
        feedback = 'no direction move'
        if cmd == -1 or cmd == 0:
            self.stop()
            feedback = 'stop'
        elif cmd == 1:
            self.go_forward()
            feedback = 'go forward'
        elif cmd == 2:
            self.go_backward()
            feedback = 'go backward'
        elif cmd == 3:
            self.go_left()
            feedback = 'go left'
        elif cmd == 4:
            self.go_right()
            feedback = 'go right'
        elif cmd == 5:
            self.go_forward_left()
            feedback = 'go forward left'
        elif cmd == 6:
            self.go_forward_right()
            feedback = 'go forward right'
        elif cmd == 7:
            self.go_backward_left()
            feedback = 'go backward left'
        elif cmd == 8:
            self.go_backward_right()
            feedback = 'go backward right'
        elif cmd == 9:
            self.turn_left()
            feedback = 'turn left'
        elif cmd == 10:
            self.turn_right()
            feedback = 'turn right'
        self.control()
        return feedback

    def arm_direction_control(self, cmd):
        result = 'no arm move'
        if cmd != -1 :
            arms = ['L1','L2','R1','R2']
            direc = ['up','down','right','left']
            result = f'arm : {arms[cmd//4]} {direc[cmd%4]}'
        return result
        
    # mode   [stop a, stop, for, back, left, right, for left, for right, bac left, bac right, t left, l right]
    # nuber  [-1    , 0   , 1  , 2   , 3   , 4    , 5       , 6        , 7       , 8        , 9     , 10]
    def stop(self): 
        self.F_L = 64
        self.F_R = 128
        self.B_L = 0
        self.B_R = 192
    def go_forward(self):
        self.F_L = 0 + self.spd
        self.F_R = 128 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 128 + self.spd
    def go_backward(self):
        self.F_L = 64 + self.spd
        self.F_R = 192 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 192 + self.spd
    def go_right(self):
        self.F_L = 0 + self.spd
        self.F_R = 192 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 128 + self.spd
    def go_left(self):
        self.F_L = 64 + self.spd
        self.F_R = 128 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 192 + self.spd
    def go_forward_left(self):
        self.F_L = 0
        self.F_R = 128 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 192
    def go_forward_right(self):
        self.F_L = 0 + self.spd
        self.F_R = 128   
        self.B_L = 0
        self.B_R = 128 + self.spd
    def go_backward_left(self):
        self.F_L = 64 + self.spd
        self.F_R = 128
        self.B_L = 0
        self.B_R = 192 + self.spd
    def go_backward_right(self):
        self.F_L = 0
        self.F_R = 192 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 128
    def turn_right(self):
        self.F_L = 0 + self.spd
        self.F_R = 192 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 192 + self.spd
    def turn_left(self):
        self.F_L = 64 + self.spd
        self.F_R = 128 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 128 + self.spd
    def control(self):
        print(f'F_L : {self.F_L}, F_R : {self.F_R}, B_L : {self.B_L}, B_R : {self.B_R}')
        packetFL = bytearray()
        packetFR = bytearray()
        packetBL = bytearray()
        packetBR = bytearray()
        packetFL.append(self.F_L)
        packetFR.append(self.F_R)
        packetBL.append(self.B_L)
        packetBR.append(self.B_R)
        ss.write(packetFL)
        ss.write(packetFR)
        s.write(packetBL)
        s.write(packetBR)
        # print(f'received massage!')

def main(args=None):
    rclpy.init()
    server = TestMotorControl()
    print(f'JOY Control server is running...')
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print(f'driver service server terminated!')
        server.destroy_node()

if __name__ == '__main__':
    main()