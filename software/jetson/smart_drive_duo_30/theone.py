#!/usr/bin/env python3 
import rclpy
import json
import time
from rclpy.node import Node
from std_msgs.msg import Float32

import serial.tools.list_ports

# max RPM = 400 (0.4 rev/min) -> @60 smart_drive_duo_30 
PPR = 330
kp = 0.4
ki = 0.001
kd = 0.08

class TestMotorControl(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.sub = self.create_subscription(Float32, 'cmd_rpm', self.cmd_rpm, 10)
        
        self.F_L = 0
        self.F_R = 0
        self.B_L = 0
        self.B_R = 0
        self.spd = 0

        self.prevPFL = 0
        self.prevPFR = 0
        self.prevPRL = 0
        self.prevPRR = 0    
        self.prevRPM_time = int(time.time() * 1000) # millisecond
        
        self.prevTick_FL = 0
        self.prevTime_FL = 0
        self.prev_err_FL = 0
        
        self.desiredRPM_FL = 0
        self.measuredRPM_FL = 0
        self.desiredSPD_FL = 0
        
        for port in serial.tools.list_ports.comports():
            print(port.device)
        
        self.s = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ss = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)

    def cmd_rpm(self, msg):
        self.desiredRPM_FL = msg.data
        self.desiredSPD_FL = self.FL_rpm_to_spd(self.desiredRPM_FL)
        
    def FL_rpm_to_spd(self, rpm):
        return int(round(rpm / 6))
    
    def FL_control(self, fl_spd_err):
        realspd = self.desiredSPD_FL
        realspd += fl_spd_err
        realspd = min(realspd, 61)
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

            if 'FL_cFL' in response_data:
                realP_FL = response_data['FL_cFL']
                
            if 'FR_cFR' in response_data:
                realP_FR = response_data['FR_cFR']

            if 'RL_cRL' in response_data:
                realP_RL = response_data['RL_cRL']
                
            if 'RR_cRR' in response_data:
                realP_RR = response_data['RR_cRR']    
            
            self.counterRPM(realP_FL, realP_FR, realP_RL, realP_RR)
            
            spd_err = self.FL_pid_err(self.desiredRPM_FL, self.measuredRPM_FL)
            self.FL_control(spd_err)

            print(f"realP_FL : {realP_FL}, realP_FR : {realP_FR}, realP_RL : {realP_RL}, realP_RR : {realP_RR}")
            
            return response_data
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON response: {e}")
            return None


    def counterRPM(self, inpFL, inpFR, inpRL, inpRR):
        curPFL = inpFL
        curPFR = inpFR
        curPRL = inpRL
        curPRR = inpRR
        curRPM_time = int(time.time() * 1000)
        diffRPM_time = curRPM_time - self.prevRPM_time 
        RPM_Fl = round((((curPFL - self.prevPFL) / PPR) / (diffRPM_time * 0.001)) * 60, 2) 
        RPM_Fr = round((((curPFR - self.prevPFR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rl = round((((curPRL - self.prevPRL) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rr = round((((curPRR - self.prevPRR) / PPR) / (diffRPM_time * 0.001)) * 60, 2)
        
        self.prevPFL = curPFL
        self.prevPFR = curPFR
        self.prevPRL = curPRL
        self.prevPRR = curPRR
        
        self.prevRPM_time = curRPM_time
        
        print(f'm_rpmFL : {RPM_Fl}, m_rpmFR : {RPM_Fr}, m_rpmRL : {RPM_Rl}, m_rpmRR : {RPM_Rr}')
        
        self.measuredRPM_FL = RPM_Fl
        self.measuredRPM_FR = RPM_Fr
        self.measuredRPM_RL = RPM_Rl
        self.measuredRPM_RR = RPM_Rr
        
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
