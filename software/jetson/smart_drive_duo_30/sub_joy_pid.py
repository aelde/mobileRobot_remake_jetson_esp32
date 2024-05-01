#!/usr/bin/env python3 
import rclpy
import json
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
serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)


# ---------------------------
# input : encoder(ppr)

# // PID control parameters
kp = 0.4;
ki = 0.001;
kd = 0.08;

# //Motor pulse parameters
# float PPR = 840 * 4;
# long pl, pr;

# //Left Wheel control parameter
# unsigned long curTimeL, prevTimeL, dtL;
# long curTickL, prevTickL, diffTickL;
# double errL, prev_errL, sumErrL, dErrL, setRPML;
# double control_outL;
# double measuredRPML;
# double desiredRPMR, desiredRPML;

# /*Right Wheel Control*/
# unsigned long curTimeR, prevTimeR, dtR;
# long curTickR, prevTickR, diffTickR;
# double errR, prev_errR, sumErrR, dErrR, setRPMR;
# double control_outR;
# double measuredRPMR;
PPR = 330
prevPFl = 0
prevPFr = 0
prevPRL = 0
prevPRR = 0

diffRPM_time = 0
prevTick_FL = 0
 
desiredRPMR = 0 # desired spd

prevRPM_time = 0 
prevTime_FL = 0 
prev_err_FL = 0 
sumErr_FL = 0 
dErr_FL = 0
def counterRPM(inpFL, inpFR, inpRL, inpRR): # counterRPM
    curPFl = inpFL
    curPFr = inpFR
    curPRL = inpRL
    curPRR = inpRR
    curRPM_time = int(time.time() * 1000) # millisecond
    diffRPM_time = curRPM_time - prevRPM_time 
    RPM_Fl = (((curPFl - prevPFl) / PPR) / (diffRPM_time * 0.001)) * 60 # RPM
    RPM_Fr = (((curPFr - prevPFr) / PPR) / (diffRPM_time * 0.001)) * 60
    RPM_Rl = (((curPRL - prevPRL) / PPR) / (diffRPM_time * 0.001)) * 60
    RPM_Rr = (((curPRR - prevPRR) / PPR) / (diffRPM_time * 0.001)) * 60
    
    prevPFl = round(curPFl, 1)
    prevPFr = round(curPFr , 1)
    prevPRL = round(curPRL, 1)
    prevPRR = round(curPRR, 1)
    prevRPM_time = curRPM_time
    
    print(f'FL : {RPM_Fl}, FR : {RPM_Fr}, RL : {RPM_Rl}, RR : {RPM_Rr}')
    # feedback_vel_msg.linear.x = RPM_l; // RPM_l;
    # feedback_vel_msg.linear.y = RPM_r;
    # feedback_vel_msg.angular.x = control_outL;
    # feedback_vel_msg.angular.y = control_outR;
    # feedback_vel_msg.angular.z = ki;
    
def compputePID_FL(desiredRPM_FL, measuredTick_FL, dir):
    curTick_FL = measuredTick_FL
    curtTime_FL = int(time.time() * 1000) # millisecond
    
    setPWM_FL = desiredRPM_FL
    dir_FL = dir
    
    diffTick_FL = curTick_FL - prevTick_FL
    dt_FL = curtTime_FL - prevTime_FL

    measureRPM_FL = ((diffTick_FL / PPR) / (dt_FL * 0.001)) * 60
    
    err_FL = abs(setPWM_FL) - abs(measureRPM_FL)
    
    dire = setPWM_FL - measureRPM_FL
    
    sumErr_FL += err_FL * dt_FL
    
    dErr_FL = (err_FL - prev_err_FL) / dt_FL
    
    i_term_FL = min(ki * sumErr_FL, 60)
    
    control_out_FL = (kp * err_FL) + i_term_FL + (kd * dErr_FL)
    if control_out_FL < 0: control_out_FL = 0.0
    
    control_out_FL = min(control_out_FL, 60)
    
    prev_err_FL = err_FL
    prevTick_FL = curTick_FL
    prevTime_FL = curtTime_FL
    
    return control_out_FL
# ------------------------
class TestMotorControl(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
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
        r = self.send_json_and_receive_response()
        if r :
            print("Received response:")
            print(json.dumps(r, indent=2))
        counterRPM(self.F_L, self.F_R, self.B_L, self.B_R)
            
# mode   [stop a, stop, for, back, left, right, for left, for right, bac left, bac right, t left, l right]
# nuber  [-1    , 0   , 1  , 2   , 3   , 4    , 5       , 6        , 7       , 8        , 9     , 10]
    def car_direction_control(self, cmd):
        feedback = 'no direction move'
        if cmd == -1 or cmd == 0:
            self.stop()
            feedback = 'stop'
        elif cmd == 1:
            self.go_backward()
            feedback = 'go forward'
        elif cmd == 2:
            self.go_forward()
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

    # mode   [stop a, stop, for, back, left, right, for left, for right, bac left, bac right, t left, l right]
    # nuber  [-1    , 0   , 1  , 2   , 3   , 4    , 5      , 6        , 7       , 8        , 9     , 10]
    def stop(self): 
        self.F_L = 64
        self.F_R = 128
        self.B_L = 0
        self.B_R = 192
    def go_forward(self):
        self.F_L = 64 + self.spd
        self.F_R = 192 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 192 + self.spd
    def go_backward(self):
       self.F_L = 0 + self.spd
       self.F_R = 128 + self.spd
       self.B_L = 0 + self.spd
       self.B_R = 128 + self.spd
    def go_left(self):
        self.F_L = 0 + self.spd
        self.F_R = 192 + self.spd
        self.B_L = 64 + self.spd
        self.B_R = 128 + self.spd
    def go_right(self):
        self.F_L = 64 + self.spd
        self.F_R = 128 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 192 + self.spd
    def go_backward_left(self):
        self.F_L = 0
        self.F_R = 128 + self.spd
        self.B_L = 0 + self.spd
        self.B_R = 192
    def go_backward_right(self):
        self.F_L = 0 + self.spd
        self.F_R = 128   
        self.B_L = 0
        self.B_R = 128 + self.spd
    def go_forward_left(self):
        self.F_L = 64 + self.spd
        self.F_R = 128
        self.B_L = 0
        self.B_R = 192 + self.spd
    def go_forward_right(self):
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
        
    def send_json_and_receive_response(self):
        # Read response from ESP32
       
        response = serial_port.readline().decode('utf-8').strip()

        try:
            # Parse the received JSON response
            response_data = json.loads(response)
            # print(f"Received response: {response_data}")
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