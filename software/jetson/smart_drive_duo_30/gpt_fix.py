#!/usr/bin/env python3 
import rclpy
import json
import time
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray, Float32

import serial.tools.list_ports

# max RPM = 400 (0.4 rev/min) -> @60 smart_drive_duo_30 
PPR = 330
kp = 0.4
ki = 0.001
kd = 0.08
class TestMotorControl(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.sub = self.create_subscription(Int8MultiArray, 'joystick_topic', self.update_control, 10)
        # self.sub_cmd_rpm = self.create_subscription(Int8MultiArray, 'cmd_rpm_dir', self.update_cmd_rpm_dir, 10)
        self.sub_rpm = self.create_subscription(Float32, 'cmd_rpm', self.cmd_rpm, 10)
        
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
        
        #RPM FL
        self.prevTick_FL = 0
        self.prevTime_FL = 0
        self.prev_err_FL = 0
        
        #desiredRPM
        self.desiredRPM_FL = 0
        self.desiredRPM_FR = 0
        self.desiredRPM_RL = 0
        self.desiredRPM_RR = 0
        
        #desiredSPD
        self.desiredSPD_FL = 0
        self.desiredSPD_FR = 0
        self.desiredSPD_RL = 0
        self.desiredSPD_RR = 0
        
        for port in serial.tools.list_ports.comports():
            print(port.device)
        
        self.s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ss = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)

    def update_control(self, msg):
        for i, j in enumerate(msg.data):
            if i == 0: 
                self.spd = j 
                self.get_logger().info(f'spd : {self.spd}')
            if i == 1:
                self.get_logger().info(f'car : {self.car_direction_control(j)}')

        r = self.send_json_and_receive_response()
        if r:
            print("Received response:")
            print(json.dumps(r, indent=2))
            
        
    def cmd_rpm(self, msg):
        self.desiredRPM_FL = msg.data
        self.desiredSPD_FL = self.FL_rpm_to_spd(self.desiredRPM_FL)
        
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
        
        self.control()
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
    
    def computePID_FL(self, desiredRPM_FL, measuredTick_FL):
        sumErr_FL = 0
        curTick_FL = measuredTick_FL  
        curTime = int(time.time() * 1000) # millisecond
        
        setPoint = desiredRPM_FL
        
        diffTick_FL = curTick_FL - self.prevTick_FL
        dt_FL = curTime - self.prevTime_FL
        
        measureRPM_FL = ((diffTick_FL / PPR) / (dt_FL * 0.001)) * 60
        
        err_FL = abs(setPoint) - abs(measureRPM_FL)
        
        sumErr_FL += err_FL * dt_FL
        
        dErr_FL = (err_FL - self.prev_err_FL) / dt_FL
        
        i_term_FL = min(ki * sumErr_FL, 200)
        
        control_out_FL = (kp * err_FL) + i_term_FL + (kd * dErr_FL)
        if control_out_FL < 0: control_out_FL = 0.0
        
        control_out_FL = min(control_out_FL, 255)
        
        self.prev_err_FL = err_FL
        self.prevTick_FL = curTick_FL
        self.prevTime_FL = curTime
        
        print(f'real_rpm_FL : {round(measureRPM_FL,2)} setpoint_FL : {round(setPoint,2)} pid_FL : {round(control_out_FL,2)}')
        print(f'error_FL : {round(err_FL,2)} sumErr_FL : {round(sumErr_FL,2)}')
        
        print(f'cal spd : {self.FL_rpm_to_spd(measureRPM_FL)}')
        # print('pid FL : ', round(control_out_FL,2))
        return control_out_FL

    def FL_pid_err(self, desiredRPM_FL, measuredTick_FL):
        pid_spd = 0
        curTick_FL = measuredTick_FL  
        curTime = int(time.time() * 1000) # millisecond
        
        setPoint = desiredRPM_FL
        
        diffTick_FL = curTick_FL - self.prevTick_FL
        dt_FL = curTime - self.prevTime_FL
        
        measureRPM_FL = ((diffTick_FL / PPR) / (dt_FL * 0.001)) * 60
        
        err_FL = setPoint - measureRPM_FL
        err_spd =self.FL_rpm_to_spd(abs(err_FL))
        if err_FL < 0:
            pid_spd += err_spd
        else: pid_spd -= err_spd
        
        self.prevTick_FL = curTick_FL
        self.prevTime_FL = curTime
        
        print(f'real_rpm_FL : {round(measureRPM_FL,2)} setpoint_FL : {round(setPoint,2)}')
        print(f'error_FL : {round(err_FL,2)} ')
        
        print(f'cal spd : {self.FL_rpm_to_spd(measureRPM_FL)}')
        # print('pid FL : ', round(control_out_FL,2))
        return pid_spd
    def FL_rpm_to_spd(self, rpm):
        # 6 rpm = 1 spd
        return int(round(rpm / 6))
    def FL_spd_to_rpm(self, spd):
        # 1 spd = 6 rpm
        return spd * 6
    def FL_control(self, fl_spd_err):
        realspd = self.desiredSPD_FL
        realspd + fl_spd_err
        packet = bytearray([realspd])
        self.ss.write(packet)
        
    def control(self):
        packetFL = bytearray([self.F_L])
        packetFR = bytearray([self.F_R])
        packetBL = bytearray([self.B_L])
        packetBR = bytearray([self.B_R])
        self.ss.write(packetFL)
        self.ss.write(packetFR)
        self.s.write(packetBL)
        self.s.write(packetBR)
        
    def send_json_and_receive_response(self):
        response = self.serial_port.readline().decode('utf-8').strip()
        try:
            response_data = json.loads(response)
            print("Received response:")
            print(json.dumps(response_data, indent=2))

            # Access specific keys from the JSON object
            if 'FL_cFL' in response_data:
                realP_FL = response_data['FL_cFL']
                # print(f"Value for key2: {value2}")
                
            if 'FR_cFR' in response_data:
                realP_FR = response_data['FR_cFR']
                # print(f"Value for key1: {value1}")

            if 'RL_cRL' in response_data:
                realP_RL = response_data['RL_cRL']
                
            if 'RR_cRR' in response_data:
                realP_RR = response_data['RR_cRR']    
            
            
            self.counterRPM(realP_FL, realP_FR, realP_RL, realP_RR)
            
            self.computePID_FL(self.desiredRPM_FL, realP_FL)
            
            # spd_err = self.FL_pid_err(self.desiredRPM_FL, realP_FL)
            # self.FL_control(spd_err)
            

            print(f"realP_FL : {realP_FL}, realP_FR : {realP_FR}, realP_RL : {realP_RL}, realP_RR : {realP_RR}")
            
            return response_data
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON response: {e}")
            return None


    def counterRPM(self, inpFL, inpFR, inpRL, inpRR):
        # Implement your RPM counter logic here...
        curPFL = inpFL
        curPFR = inpFR
        curPRL = inpRL
        curPRR = inpRR
        curRPM_time = int(time.time() * 1000) # millisecond
        diffRPM_time = curRPM_time - self.prevRPM_time 
        RPM_Fl = round((((curPFL - self.prevPFL) / PPR) / (diffRPM_time * 0.001)) * 60, 2) # RPM
        RPM_Fr = round((((curPFR - self.prevPFR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rl = round((((curPRL - self.prevPRL) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rr = round((((curPRR - self.prevPRR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        
        self.prevPFL = curPFL
        self.prevPFR = curPFR
        self.prevPRL = curPRL
        self.prevPRR = curPRR
        
        self.prevRPM_time = curRPM_time
        
        print(f'prmFL : {RPM_Fl}, rpmFR : {RPM_Fr}, rpmRL : {RPM_Rl}, rpmRR : {RPM_Rr}')

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
