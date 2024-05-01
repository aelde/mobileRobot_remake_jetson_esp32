#!/usr/bin/env python3 
import rclpy
import json
import time
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32, Int32

import serial.tools.list_ports

# max RPM = 400 (0.4 rev/min) -> @60 smart_drive_duo_30 
PPR = 330
# // PID control parameters
kp = 0.4
ki = 0.001
kd = 0.08

class TestMotorControl(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        # self.sub = self.create_subscription(Int8MultiArray, 'joystick_topic', self.update_control, 10)
        # self.sub_cmd_rpm = self.create_subscription(Int8MultiArray, 'cmd_rpm_dir', self.update_cmd_rpm_dir, 10)
        self.sub_rpm = self.create_subscription(Float32, 'cmd_rpm', self.cmd_rpm, 10)
        
        self.sub_measure_tick = self.create_subscription(Int32MultiArray, 'measure_tick', self.measure_tick, 10)
        
        self.pub_measure_FL = self.create_publisher(Float32, 'measure_FL', 10)
        
        self.pub_log_value = self.create_publisher(Float32, 'log_value', 10)
        self.timer = self.create_timer(0.05, self.log_value_pub)
        self.timer2 = self.create_timer(0.008, self.timer2_callback)
        
        self.F_L = 0
        self.F_R = 0
        self.B_L = 0
        self.B_R = 0
        self.spd = 0

        self.prevRPM = 0
        self.prevPFL = 0
        self.prevPFR = 0
        self.prevPRL = 0
        self.prevPRR = 0    
        self.prevRPM_time = int(time.time() * 1000) # millisecond
        
        self.sumErr_FL = 0
        
        #RPM FL
        self.prevTick_FL = 0
        self.prevTime_FL = 0
        self.prev_err_FL = 0
        
        #desiredRPM
        self.desiredRPM_FL = 0
        self.desiredRPM_FR = 0
        self.desiredRPM_RL = 0
        self.desiredRPM_RR = 0
        
        #measuredRPM
        self.measureRPM_FL = 0
        self.measureRPM_FR = 0
        self.measureRPM_RL = 0
        self.measureRPM_RR = 0
        
        self.realP_FL = 0
        self.realP_FR = 0
        self.realP_RL = 0
        self.realP_RR = 0
        
        self.RPM_FL = 0
        self.RPM_FR = 0
        self.RPM_RL = 0
        self.RPM_RR = 0   
        
        self.measureRPM_FL = 0
        
        self.control_pid_out = 0
        
        for port in serial.tools.list_ports.comports():
            print(port.device)
        
        self.s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ss = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        # self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)

    def log_value_pub(self):

        
        print(f'tFL : {self.realP_FL}  | tFR : {self.realP_FR}  | tRL : {self.realP_RL}  | tRR : {self.realP_RR}')
        print()
        # print(f'RPM_FL : {self.RPM_FL}   | RPM_FR : {self.RPM_FR} | RPM_RL : {self.RPM_RL} | RPM_RR : {self.RPM_RR}')
        # print()
        print(f'measureRPM_FL : {self.measureRPM_FL} | control_pid_out : {self.control_pid_out}')
        
        # self.counterRPM(self.realP_FL, self.realP_FR, self.realP_RL, self.realP_RR)
        # self.computePID_FL(self.desiredRPM_FL, self.realP_FL)
    
    def timer2_callback(self):
        msg = Float32()
        msg.data = float(self.measureRPM_FL)
        self.pub_measure_FL.publish(msg)
        
        self.computePID_FL(self.desiredRPM_FL, self.realP_FL)
    
    def cmd_rpm(self, msg):
        self.desiredRPM_FL = msg.data
        if msg.data== 0.0:
            self.stop()
            
        # self.computePID_FL(self.desiredRPM_FL, self.realP_FL)
        

        
    def car_direction_control(self, cmd):
        feedback = 'no direction move'
        if cmd == -1 or cmd == 0:
            self.stop()
            feedback = 'stop'
        elif cmd == 1:
            self.go_backward()
            feedback = 'go backward'
        elif cmd == 2:
            self.go_forward()
            feedback = 'go forward'
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
        
        # self.control()
        return feedback

    def stop(self): 
        self.F_L = 64
        self.F_R = 128
        self.B_L = 0
        self.B_R = 192
    # Add other direction control methods here...
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
    
    def counterRPM(self, inpFL, inpFR, inpRL, inpRR):
        # Implement your RPM counter logic here...
        curPFL = inpFL
        curPFR = inpFR
        curPRL = inpRL
        curPRR = inpRR
        curRPM_time = int(time.time() * 1000) # millisecond
        diffRPM_time = curRPM_time - self.prevRPM_time 
        
        self.RPM_FL = round((((curPFL - self.prevPFL) / PPR) / (diffRPM_time * 0.001)) * 60, 2) # RPM
        self.RPM_FR = round((((curPFR - self.prevPFR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        self.RPM_RL = round((((curPRL - self.prevPRL) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        self.RPM_RR = round((((curPRR - self.prevPRR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        
        self.prevPFL = curPFL
        self.prevPFR = curPFR
        self.prevPRL = curPRL
        self.prevPRR = curPRR
        
        self.prevRPM_time = curRPM_time
    
        
        # print(f'prmFL : {RPM_Fl}, rpmFR : {RPM_Fr}, rpmRL : {RPM_Rl}, rpmRR : {RPM_Rr}')
    
    def computePID_FL(self, desiredRPM_FL, measuredTick_FL):
        self.sumErr_FL = 0
        curTick_FL = measuredTick_FL  
        curTime = int(time.time() * 1000) # millisecond
        
        setPoint = desiredRPM_FL
        
        diffTick_FL = curTick_FL - self.prevTick_FL
        dt_FL = curTime - self.prevTime_FL
        
        measureRPM_Fl = ((diffTick_FL / PPR) / (dt_FL * 0.001)) * 60
        self.measureRPM_FL = measureRPM_Fl
        
        err_FL = abs(setPoint) - abs(measureRPM_Fl)
        
        self.sumErr_FL += err_FL * dt_FL
        
        dErr_FL = (err_FL - self.prev_err_FL) / dt_FL
        
        i_term_FL = min((ki * self.sumErr_FL), 50)
        
        control_out_FL = (kp * err_FL) + i_term_FL + (kd * dErr_FL)
        
        if control_out_FL < 0: control_out_FL = 0.0
        
        control_out_FL = min(control_out_FL, 60)
        
        self.prev_err_FL = err_FL
        self.prevTick_FL = curTick_FL
        self.prevTime_FL = curTime
        
        print()
        print(f'set_FL', {setPoint}, 'meas_FL', {round(measureRPM_Fl,2)}, 'err_FL', {round(err_FL,2)})
        print('pid FL : ', round(control_out_FL,2))
        
        
        self.spd = int(control_out_FL)
        print('speed FL : ', self.spd)
        print()
        self.go_forward()
        self.control_FL()
        
        self.control_pid_out = control_out_FL
        return control_out_FL
    
    def control_FL(self):
        packetFL = bytearray([self.F_L])
        self.ss.write(packetFL)
        
    def control(self):
        packetFL = bytearray([self.F_L])
        packetFR = bytearray([self.F_R])
        packetBL = bytearray([self.B_L])
        packetBR = bytearray([self.B_R])
        self.ss.write(packetFL)
        self.ss.write(packetFR)
        self.s.write(packetBL)
        self.s.write(packetBR)

def main(args=None):
    rclpy.init()
    server = TestMotorControl()
    print(f'motor_driver_node is running...')
    try:
        rclpy.spin(server)
        server.print_ticl_FL()
    except KeyboardInterrupt:
        print(f'motor_driver_node is terminated!')
        server.destroy_node()

if __name__ == '__main__':
    main()