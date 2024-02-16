
import pygame
from time import sleep
pygame.init()
controller = pygame.joystick.Joystick(0)
controller.init()
buttons = {'1':0,'2':0,'3':0,'4':0,
           'L1':0,'R1':0,'L2':0,'R2':0,
           'share':0,'options':0,
           'ax1':0.,'ax2':0.,'ax3':0.,'ax4':0.}
axiss=[0.,0.,0.,0.,0.,0.]
 
# param set
spd_count = 0
ARM_MODE = False
 
def getJS(name=''):
 
    global buttons
    # retrieve any events ...
    for event in pygame.event.get():                                # Analog Sticks
        if event.type == pygame.JOYAXISMOTION:
            # axiss[event.axis] = round(event.value,2)
            axiss[event.axis] = round(event.value)
        elif event.type == pygame.JOYBUTTONDOWN:                    # When button pressed
            # print(event.dict, event.joy, event.button, 'PRESSED')
            for x,(key,val) in enumerate(buttons.items()):
                if x<10:
                    if controller.get_button(x):buttons[key]=1
        elif event.type == pygame.JOYBUTTONUP:                      # When button released
            #print(event.dict, event.joy, event.button, 'released')
            for x,(key,val) in enumerate(buttons.items()):
                if x<10:
                    if event.button ==x:buttons[key]=0
 
    # to remove element 2 since axis numbers are 0 1 3 4     
    buttons['ax1'],buttons['ax2'] ,buttons['ax3'] ,buttons['ax4'] = [axiss[0],axiss[1],axiss[2],axiss[3]]
    if name == '':
        return buttons
    else:
        return buttons[name]

def spd_counter(buttons):
    global spd_count
    if not ARM_MODE: 
        if buttons['1'] == 1: spd_count += 1
        if buttons['3'] == 1: spd_count -= 1
        if spd_count < 0: spd_count = 0
        if spd_count > 62: spd_count = 62
    return spd_count
def direction_move(buttons):
    result = 'arm mode on!'
    if not ARM_MODE:
        if buttons['ax1'] == 0 and buttons['ax2'] == 0 and buttons['ax3'] == 0 and buttons['ax4'] == 0:
            # print('stop')
            result = 'stop'
        elif buttons['ax2'] == -1 and buttons['ax1'] == 0:
            # print('go forward')
            result = 'go forward'
        elif buttons['ax2'] == 1 and buttons['ax1'] == 0:
            # print('go backward')
            result = 'go backward'
        elif buttons['ax1'] == -1 and buttons['ax2'] == 0:
            # print('go left')
            result = 'go left'
        elif buttons['ax1'] == 1 and buttons['ax2'] == 0:
            # print('go right')
            result = 'go right'
        elif buttons['ax2'] == -1 and buttons['ax1'] == -1:
            # print('go forward left')
            result = 'go forward left'
        elif buttons['ax2'] == -1 and buttons['ax1'] == 1:
            # print('go forward right')
            result = 'go forward right'
        elif buttons['ax2'] == 1 and buttons['ax1'] == -1:
            # print('go backward left')
            result = 'go backward left'
        elif buttons['ax2'] == 1 and buttons['ax1'] == 1:
            # print('go backward right')
            result = 'go backward right'
        elif buttons['ax3'] == -1 and buttons['ax4'] == 0:
            # print('turn left')
            result = 'turn left'
        elif buttons['ax3'] == 1 and buttons['ax4'] == 0:
            # print('turn right')
            result = 'turn right'
    return result
def arm_mode_check(buttons):
    global ARM_MODE
    if buttons['L1'] == 1 or buttons['L2'] == 1 or buttons['R1'] == 1 or buttons['R2'] == 1: ARM_MODE = True
    else : ARM_MODE = False
    return ARM_MODE
def arm_move(buttons):
    result = 'no arm movement'
    arms = ['L1','L2','R1','R2']
    direc = ['up','down','right','left']
    b = ['1','3','2','4']
    for i in range(len(arms)):
        for j in range(len(b)):
            if buttons[arms[i]] == 1 and buttons[b[j]] == 1:
                result = f'arm {arms[i]} {direc[j]}'
    return result  
    # full arm move
    # if buttons['L1'] == 1 and buttons['1'] == 1: 
    #     print('arm 1 up')
    # if buttons['L1'] == 1 and buttons['3'] == 1:
    #     print('arm 1 down')
    # if buttons['L1'] == 1 and buttons['2'] == 1:
    #     print('arm 1 right')
    # if buttons['L1'] == 1 and buttons['4'] == 1:
    #     print('arm 1 left')
    # if buttons['L2'] == 1 and buttons['1'] == 1:
    #     print('arm 2 up')
    # if buttons['L2'] == 1 and buttons['3'] == 1:
    #     print('arm 2 down')
    # if buttons['L2'] == 1 and buttons['2'] == 1:
    #     print('arm 2 right')
    # if buttons['L2'] == 1 and buttons['4'] == 1:
    #     print('arm 2 left')
    # if buttons['R1'] == 1 and buttons['1'] == 1:
    #     print('arm 3 up')
    # if buttons['R1'] == 1 and buttons['3'] == 1:
    #     print('arm 3 down')
    # if buttons['R1'] == 1 and buttons['2'] == 1:
    #     print('arm 3 right')
    # if buttons['R1'] == 1 and buttons['4'] == 1:
    #     print('arm 3 left')
    # if buttons['R2'] == 1 and buttons['1'] == 1:
    #     print('arm 4 up')
    # if buttons['R2'] == 1 and buttons['3'] == 1:
    #     print('arm 4 down')
    # if buttons['R2'] == 1 and buttons['2'] == 1:
    #     print('arm 4 right')
    # if buttons['R2'] == 1 and buttons['4'] == 1:
    #     print('arm 4 left')
    # else:
    #     print('no arm movement')
def main():
    # print(getJS()) # To get all values
    spd_counter(getJS())
    arm_mode_check(getJS())
    print(f'car speed : {spd_count}')
    print(f'car movement : {direction_move(getJS())}')
    print(f'arm mode : {ARM_MODE}')
    print(f'arm movement : {arm_move(getJS())}')
    
    sleep(0.05)
 
if __name__ == '__main__':
  while True:
    main()