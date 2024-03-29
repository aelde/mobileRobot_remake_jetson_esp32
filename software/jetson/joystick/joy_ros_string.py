import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pygame
from time import sleep
pygame.init()

class JoystickControlPubNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.pub = self.create_publisher(String, 'joystick_topic', 10)
        self.timer = self.create_timer(0.05, self.joystick_pub)
        
        # pygame setting
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.buttons = {'1':0,'2':0,'3':0,'4':0,
                'L1':0,'R1':0,'L2':0,'R2':0,
                'share':0,'options':0,
                'ax1':0.,'ax2':0.,'ax3':0.,'ax4':0.}
        self.axiss=[0.,0.,0.,0.,0.,0.]
        
        # param setting
        self.spd = 0
        self.arm_mode = False
        self.car_direction = ''
        self.arm_movement = ''
        
    def joystick_pub(self):
        # msg = String()
        # msg.data = f'heelo : {self.c}'
        # self.pub.publish(msg)
        # self.c += 1   
        print(self.buttons)
        self.getJS()
        self.arm_mode_check()
        self.spd_counter()
        self.direction_move()
        self.arm_move()
        
        msg = String()
        msg.data = f'spd : {self.spd} , car_direc : {self.car_direction} , arm_move : {self.arm_movement}'  
        self.pub.publish(msg)
              
    def getJS(self, name=''):
        # retrieve any events ...
        for event in pygame.event.get():                                # Analog Sticks
            if event.type == pygame.JOYAXISMOTION:
                # self.axiss[event.axis] = round(event.value,2)
                self.axiss[event.axis] = round(event.value)
            elif event.type == pygame.JOYBUTTONDOWN:                    # When button pressed
                # print(event.dict, event.joy, event.button, 'PRESSED')
                for x,(key,val) in enumerate(self.buttons.items()):
                    if x<10:
                        if self.controller.get_button(x):self.buttons[key]=1
            elif event.type == pygame.JOYBUTTONUP:                      # When button released
                #print(event.dict, event.joy, event.button, 'released')
                for x,(key,val) in enumerate(self.buttons.items()):
                    if x<10:
                        if event.button ==x:self.buttons[key]=0
        # to remove element 2 since axis numbers are 0 1 3 4     
        self.buttons['ax1'],self.buttons['ax2'] ,self.buttons['ax3'] ,self.buttons['ax4'] = [self.axiss[0],self.axiss[1],self.axiss[2],self.axiss[3]]
        if name == '':
            return self.buttons
        else:
            return self.buttons[name]
    def spd_counter(self):
        if not self.arm_mode : 
            if self.spd < 1: self.spd = 0
            if self.spd > 61: self.spd = 62
            if self.buttons['1'] == 1: self.spd += 1
            if self.buttons['3'] == 1: self.spd -= 1
        return self.spd
    def direction_move(self):
        result = 'stop arm mode on!'
        if not self.arm_mode:
            if self.buttons['ax1'] == 0 and self.buttons['ax2'] == 0 and self.buttons['ax3'] == 0 and self.buttons['ax4'] == 0:
                # print('stop')
                result = 'stop'
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == 0:
                # print('go forward')
                result = 'go forward'
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == 0:
                # print('go backward')
                result = 'go backward'
            elif self.buttons['ax1'] == -1 and self.buttons['ax2'] == 0:
                # print('go left')
                result = 'go left'
            elif self.buttons['ax1'] == 1 and self.buttons['ax2'] == 0:
                # print('go right')
                result = 'go right'
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == -1:
                # print('go forward left')
                result = 'go forward left'
            elif self.buttons['ax2'] == -1 and self.buttons['ax1'] == 1:
                # print('go forward right')
                result = 'go forward right'
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == -1:
                # print('go backward left')
                result = 'go backward left'
            elif self.buttons['ax2'] == 1 and self.buttons['ax1'] == 1:
                # print('go backward right')
                result = 'go backward right'
            elif self.buttons['ax3'] == -1 and self.buttons['ax4'] == 0:
                # print('turn left')
                result = 'turn left'
            elif self.buttons['ax3'] == 1 and self.buttons['ax4'] == 0:
                # print('turn right')
                result = 'turn right'
        self.car_direction = result
        return self.car_direction 
    def arm_mode_check(self):
        if self.buttons['L1'] == 1 or self.buttons['L2'] == 1 or self.buttons['R1'] == 1 or self.buttons['R2'] == 1: 
            self.arm_mode = True
        else : self.arm_mode = False
        return self.arm_mode
    def arm_move(self):
        result = 'no arm movement'
        arms = ['L1','L2','R1','R2']
        direc = ['up','down','right','left']
        b = ['1','3','2','4']
        for i in range(len(arms)):
            for j in range(len(b)):
                if self.buttons[arms[i]] == 1 and self.buttons[b[j]] == 1:
                    result = f'arm {arms[i]} {direc[j]}'
        self.arm_movement = result
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