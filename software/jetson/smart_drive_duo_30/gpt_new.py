#!/usr/bin/env python3 
import rclpy
import json
import time
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray, Float32, Int32

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
        self.sub_rpm_FL = self.create_subscription(Float32, 'cmd_rpm_FL', self.wheel_cmd_rpm_FL, 10)
        self.sub_rpm_FR = self.create_subscription(Float32, 'cmd_rpm_FR', self.wheel_cmd_rpm_FR, 10)
        
        self.pub_FL_tick = self.create_publisher(Int32, 'FL_tick', 10)
        self.pub_FR_tick = self.create_publisher(Int32, 'FR_tick', 10)
        self.pub_RL_tick = self.create_publisher(Int32, 'RL_tick', 10)
        self.pub_RR_tick = self.create_publisher(Int32, 'RR_tick', 10)
        
        self.FL_tick = Int32()
        self.FR_tick = Int32()
        self.RL_tick = Int32()
        self.RR_tick = Int32()
        
        self.timer = self.create_timer(0.1, self.pub_tick_callback)
        
        
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

        #measuredRPM
        self.measureRPM_FL = 0
        self.measureRPM_FR = 0
        self.measureRPM_RL = 0
        self.measureRPM_RR = 0
        
        #desiredSPD
        self.desiredSPD_FL = 0
        self.desiredSPD_FR = 0
        self.desiredSPD_RL = 0
        self.desiredSPD_RR = 0
        
        #measuredtick
        self.measureTick_FL = 0
        self.measureTick_FR = 0
        self.measureTick_RL = 0
        self.measureTick_RR = 0

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
    
    def pub_tick_callback(self):
        self.FL_tick.data = self.measureTick_FL
        self.pub_FL_tick.publish(self.FL_tick)
        
        self.FR_tick.data = self.measureTick_FR
        self.pub_FR_tick.publish(self.FR_tick)
        
        self.RL_tick.data = self.measureTick_RL
        self.pub_RL_tick.publish(self.RL_tick)
        
        self.RR_tick.data = self.measureTick_RR
        self.pub_RR_tick.publish(self.RR_tick)

    def wheel_cmd_rpm_FL(self, msg):
        self.desiredRPM_FL = msg.data
        self.desiredSPD_FL = self.FL_rpm_to_spd(self.desiredRPM_FL)
        
        self.desiredRPM_RL = self.desiredRPM_FL
        self.desiredSPD_RL = self.RL_rpm_to_spd(self.desiredRPM_RL)
        
    def wheel_cmd_rpm_FR(self, msg):
        self.desiredRPM_FR = msg.data
        self.desiredSPD_FR = self.FR_rpm_to_spd(self.desiredRPM_FR)
        
        self.desiredRPM_RR = self.desiredRPM_FR
        self.desiredSPD_RR = self.RR_rpm_to_spd(self.desiredRPM_RR)    
        
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
            pid_spd = err_spd
        else: pid_spd = -err_spd
        
        self.prevTick_FL = curTick_FL
        self.prevTime_FL = curTime
        
        print(f'real_rpm_FL : {round(measureRPM_FL,2)} setpoint_FL : {round(setPoint,2)}')
        print(f'error_FL : {round(err_FL,2)} ')
        
        print(f'cal spd : {self.FL_rpm_to_spd(measureRPM_FL)}')
        # print('pid FL : ', round(control_out_FL,2))
        
        # packet = bytearray([sent_spd])
        # self.ss.write(packet)
        
        # self.measuredSPD_FL += pid_spd
        
        
        return pid_spd
    
    def FL_pid_err_2(self, desiredRPM_FL, measuredRPM_FL):
        pid_spd = 0
        
        setPoint = desiredRPM_FL
        
        err_FL = setPoint - measuredRPM_FL
        err_spd =self.FL_rpm_to_spd(abs(err_FL))
        if err_FL < 0:
            pid_spd = err_spd
        else: pid_spd = -err_spd
        
        # self.prevTick_FL = curTick_FL
        # self.prevTime_FL = curTime
        
        # print(f'real_rpm_FL : {round(measuredRPM_FL,2)} setpoint_FL : {round(setPoint,2)}')
        # print(f'error_FL : {round(err_FL,2)} ')
        
        # print(f'cal spd : {self.FL_rpm_to_spd(measuredRPM_FL)}')
        
        return pid_spd
    
    def FR_pid_err(self, desiredRPM_FR, measuredRPM_FR):
        pid_spd = 0
        
        setPoint = desiredRPM_FR
        
        err_FL = setPoint - measuredRPM_FR
        err_spd =self.FR_rpm_to_spd(abs(err_FL))
        if err_FL < 0:
            pid_spd = err_spd
        else: pid_spd = -err_spd
        
        # self.prevTick_FL = curTick_FL
        # self.prevTime_FL = curTime
        
        # print(f'real_rpm_FR : {round(measuredRPM_FR,2)} setpoint_FR: {round(setPoint,2)}')
        # print(f'error_FL : {round(err_FL,2)} ')
        
        # print(f'cal spd : {self.FR_rpm_to_spd(measuredRPM_FR)}')
        
        return pid_spd
    
    def RL_pid_err(self, desiredRPM_RL, measuredRPM_RL):
        pid_spd = 0
        
        setPoint = desiredRPM_RL
        
        err_FL = setPoint - measuredRPM_RL
        err_spd =self.RL_rpm_to_spd(abs(err_FL))
        if err_FL < 0:
            pid_spd = err_spd
        else: pid_spd = -err_spd
        
        # self.prevTick_FL = curTick_FL
        # self.prevTime_FL = curTime
        
        # print(f'real_rpm_FR : {round(measuredRPM_RL,2)} setpoint_FR: {round(setPoint,2)}')
        # print(f'error_FL : {round(err_FL,2)} ')
        
        # print(f'cal spd : {self.FR_rpm_to_spd(desiredRPM_RL)}')
        
        return pid_spd
    
    def RR_pid_err(self, desiredRPM_RR, measuredRPM_RR):
        pid_spd = 0
        
        setPoint = desiredRPM_RR
        
        err_FL = setPoint - measuredRPM_RR
        err_spd =self.FR_rpm_to_spd(abs(err_FL))
        if err_FL < 0:
            pid_spd = err_spd
        else: pid_spd = -err_spd
        
        # self.prevTick_FL = curTick_FL
        # self.prevTime_FL = curTime
        
        # print(f'real_rpm_FR : {round(measuredRPM_RR,2)} setpoint_FR: {round(setPoint,2)}')
        # print(f'error_FL : {round(err_FL,2)} ')
        
        # print(f'cal spd : {self.FR_rpm_to_spd(desiredRPM_RR)}')
        
        return pid_spd
    
    def FL_rpm_to_spd(self, rpm):
        # 6 rpm = 1 spd
        return int(round(rpm / 6))
    def FL_spd_to_rpm(self, spd):
        # 1 spd = 6 rpm
        return spd * 6
    def FL_control(self, fl_spd_err):
        realspd = self.desiredSPD_FL 
        spd = abs(realspd)
        print(f'real spd FL: {realspd}', end=' ')
        sentspd = abs(max(min(spd, 61),0))
        # print(f'sent spd FL: {sentspd}', )
        if realspd < 0: sentspd = sentspd + 64
        else : sentspd = sentspd
        print(f'final spd FL: {sentspd}')
        packet = bytearray([sentspd])
        self.ss.write(packet)
        
    def FR_rpm_to_spd(self, rpm):
        # 5.5 rpm = 1 spd 0nly FL wheel
        return int(round(rpm / 5.3))
    def FR_spd_to_rpm(self, spd):
        # 1 spd = 5.5 rpm
        return spd * 5.3
    def FR_control(self, fr_spd_err):
        realspd = self.desiredSPD_FR 
        spd = abs(realspd)
        print(f'real spd FR: {realspd}', end=' ')
        sentspd = abs(max(min(spd, 61),0))
        # print(f'sent spd FR: {sentspd}')
        if realspd < 0: sentspd = sentspd + 64 + 128
        else : sentspd = sentspd + 128
        # sentspd = 140
        # sentspd = 222
        print(f'fianl spd FR: {sentspd}')
        packet = bytearray([sentspd])
        self.ss.write(packet)
        
    def RL_rpm_to_spd(self, rpm):
        # 5.5 rpm = 1 spd 0nly FL wheel
        return int(round(rpm / 6))
    def RL_spd_to_rpm(self, spd):
        # 1 spd = 5.5 rpm
        return spd * 6
    def RL_control(self, rl_spd_err):
        realspd = self.desiredSPD_RL 
        spd = abs(realspd)
        print(f'real spd RL: {realspd}', end=' ')
        sentspd = max(min(spd, 61), 0)
        print(f'sent spd RL: {sentspd}', end=' ')
        if realspd < 0:
            sentspd += 64
        print(f'final spd RL: {sentspd}')
        packetRL = bytearray([sentspd])
        self.s.write(packetRL)
    
    def RR_rpm_to_spd(self, rpm):
        # 5.5 rpm = 1 spd 0nly FL wheel
        return int(round(rpm / 6))
    def RR_spd_to_rpm(self, spd):
        # 1 spd = 5.5 rpm
        return spd * 6
    def RR_control(self, rr_spd_err):
        realspd = self.desiredSPD_RR 
        spd = abs(realspd)
        print(f'real spd RR: {realspd}', end=' ')
        sentspd = max(min(spd, 61), 0)
        print(f'sent spd RR: {sentspd}', end=' ')
        if realspd < 0:
            sentspd += 128 + 64
        else:
            sentspd += 128
        print(f'final spd RR: {sentspd}')
        packetRR = bytearray([sentspd])
        self.s.write(packetRR)
        
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
            
            
            self.measureTick_FL = realP_FL
            self.measureTick_FR = realP_FR
            self.measureTick_RL = realP_RL
            self.measureTick_RR = realP_RR
            
            self.counterRPM(realP_FL, realP_FR, realP_RL, realP_RR)
            
            # self.computePID_FL(self.desiredRPM_FL, realP_FL)
            
            spd_err_FL = self.FL_pid_err_2(self.desiredRPM_FL, self.measuredRPM_FL)
            spd_err_FR = self.FR_pid_err(self.desiredRPM_FR, self.measuredRPM_FR)
            spd_err_RL = self.RL_pid_err(self.desiredRPM_RL, self.measuredRPM_RL)
            spd_err_RR = self.RR_pid_err(self.desiredRPM_RR, self.measuredRPM_RR)
            
            print(f'spEr_FL : {spd_err_FL}, spEr_FR : {spd_err_FR}, spEr_RL : {spd_err_RL}, spEr_RR : {spd_err_RR}')
            self.FL_control(spd_err_FL)
            self.FR_control(spd_err_FR)
            self.RL_control(spd_err_RL)
            self.RR_control(spd_err_RR)
            print()

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
