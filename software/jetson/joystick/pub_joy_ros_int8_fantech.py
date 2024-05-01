
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
# int8 = -128 - 127
import pygame
from time import sleep
pygame.init()

class JoystickControlPubNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.pub = self.create_publisher(Int8MultiArray, 'joystick_topic', 10)
        self.timer = self.create_timer(0.05, self.joystick_pub)
        
        # pygame setting for fintech wgp12 
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.buttons = {'A':0,   'B':0,   'f2':0,   'X':0,   'Y':0,
                   'f5':0,'L2':0,  'R2':0,     'L1':0,  'R1':0,
                   'left':0,   'right':0,  'center':0,
                   'ax0':0.,'ax1':0.,'ax2':0.,'ax3':0.,'ax4':0.,'ax5':0.,
                   'hatx':0,   'haty':0}
        self.axiss = [0.,0.,0.,0.,0.,0.]
        # self.hats = [0,0]
        
        # param setting
        self.spd = 0
        self.arm_mode = False
        self.arm_OE = 1
        self.auto_mode = True
        self.auto_move = 0
        self.toggle_arm_OE = False
        self.car_direction = ''
        self.arm_movement = ''
        
    def joystick_pub(self):
        # msg = String()
        # msg.data = f'heelo : {self.c}'
        # self.pub.publish(msg)
        # self.c += 1   
        print(self.buttons)
        self.getJS()
        # self.arm_OE_check()
        # self.arm_autoMode_check()
        # self.arm_mode_check()
        # self.spd_counter()
        # self.direction_move()
        # self.arm_move()
        
        print(f'spd : {self.spd} , car_direc : {self.car_direction} , arm_move : {self.arm_movement}, arm_OE : {self.arm_OE} ,auto_mode : {self.auto_mode}, auto_move : {self.auto_move}')
        msg = Int8MultiArray()
        msg.data = [self.spd, self.car_direction, self.arm_movement, self.arm_OE, self.auto_move]
        self.pub.publish(msg)
              
    def getJS(self, name=''):
        # retrieve any events ...
        for event in pygame.event.get():                                # Analog Sticks
            if event.type == pygame.JOYAXISMOTION:
                # self.axiss[event.axis] = round(event.value,2)
                self.axiss[event.axis] = round(event.value)
            elif event.type == pygame.JOYHATMOTION:
                # print(event.dict, event.joy, event.hat, event.value)
                hats = event.value
                self.buttons['hatx'] = hats[0]
                self.buttons['haty'] = hats[1]
            elif event.type == pygame.JOYBUTTONDOWN:                    # When button pressed
                # print(event.dict, event.joy, event.button, 'PRESSED')
                for x,(key,val) in enumerate(self.buttons.items()):
                    if x<13:
                        if self.controller.get_button(x):self.buttons[key]=1
            elif event.type == pygame.JOYBUTTONUP:                      # When button released
                #print(event.dict, event.joy, event.button, 'released')
                for x,(key,val) in enumerate(self.buttons.items()):
                    if x<13:
                        if event.button ==x:self.buttons[key]=0
        # to remove element 2 since axis numbers are 0 1 3 4     
        self.buttons['ax0'],self.buttons['ax1'] ,self.buttons['ax2'] ,self.buttons['ax3'] ,self.buttons['ax4'] ,self.buttons['ax5']= [self.axiss[0],self.axiss[1],self.axiss[2],self.axiss[3],self.axiss[4],self.axiss[5]]
        if name == '':
            return self.buttons
        else:
            return self.buttons[name]
    def spd_counter(self):
        if not self.arm_mode : 
            if self.spd < 1: self.spd = 1
            if self.spd > 61: self.spd = 61
            if self.buttons['1'] == 1: self.spd += 1
            if self.buttons['3'] == 1: self.spd -= 1
        return self.spd
    
    # mode   [stop a, stop, for, back, left, right, for left, for right, bac left, bac right, t left, l right]
    # nuber  [-1    , 0   , 1  , 2   , 3   , 4    , 5       , 6        , 7       , 8        , 9     , 10]
    
    def direction_move(self):
        result = 'stop arm mode on!'
        r = -1
        if not self.arm_mode:
            if self.buttons['ax1'] == 0 and self.buttons['ax2'] == 0 and self.buttons['ax3'] == 0 and self.buttons['ax4'] == 0:
                # print('stop')
               result = 'stop'
               r = 0
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == 0:
                # print('go forward')
                result = 'go forward'
                r = 1
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == 0:
                # print('go backward')
                result = 'go backward'
                r = 2
            elif self.buttons['ax1'] == -1 and self.buttons['ax2'] == 0:
                # print('go left')
                result = 'go left'
                r = 3
            elif self.buttons['ax1'] == 1 and self.buttons['ax2'] == 0:
                # print('go right')
                result = 'go right'
                r = 4
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == -1:
                # print('go forward left')
                result = 'go forward left'
                r = 5
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == 1:
                # print('go forward right')
                result = 'go forward right'
                r = 6
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == -1:
                # print('go backward left')
                result = 'go backward left'
                r = 7
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == 1:
                # print('go backward right')
                result = 'go backward right'
                r = 8
            elif self.buttons['ax3'] == -1 and self.buttons['ax4'] == 0:
                # print('turn left')
                result = 'turn left'
                r = 9
            elif self.buttons['ax3'] == 1 and self.buttons['ax4'] == 0:
                # print('turn right')
                result = 'turn right'
                r = 10
        self.car_direction = r
        # self.car_direction = result
        return self.car_direction 
    def arm_mode_check(self):
        if self.buttons['L1'] == 1 or self.buttons['L2'] == 1 or self.buttons['R1'] == 1 or self.buttons['R2'] == 1: 
            self.arm_mode = True
        else : self.arm_mode = False
        return self.arm_mode
    def arm_OE_check(self):
        if self.buttons['options'] == 1 and self.buttons['L1'] == 1: 
            self.arm_OE = 0
            self.auto_mode = False
        if self.buttons['share'] == 1 and self.buttons['L1'] == 1: 
            self.arm_OE = 1
            self.auto_mode = True
        return self.arm_OE , self.auto_mode
    def arm_autoMode_check(self):
        if self.auto_mode:
            if self.buttons['L1'] == 1 and self.buttons['1'] == 1: 
                self.auto_move = 0
            if self.buttons['L1'] == 1 and self.buttons['2'] == 1: 
                self.auto_move = 1
            if self.buttons['L1'] == 1 and self.buttons['3'] == 1: 
                self.auto_move = 2
            if self.buttons['L1'] == 1 and self.buttons['4'] == 1: 
                self.auto_move = 3
            return self.auto_move
    def arm_move(self):
        self.arm_movement = -1
        if not self.auto_mode:
            result = 'no arm movement'
            r = -1
            arms = ['L1','L2','R1','R2']
            direc = ['up','down','right','left']
            b = ['1','3','2','4']
            for i in range(len(arms)):
                for j in range(len(b)):
                    if self.buttons[arms[i]] == 1 and self.buttons[b[j]] == 1:
                        result = f'arm {arms[i]} {direc[j]}'
                        r = i*4 + j
            self.arm_movement = r
        return self.arm_movement
            
def main(args=None):
	rclpy.init()
	my_pub = JoystickControlPubNode()
	print("JoyStick Node is Running...")

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		print("Terminating Node...")
		my_pub.destroy_node()


if __name__ == '__main__':
	main()