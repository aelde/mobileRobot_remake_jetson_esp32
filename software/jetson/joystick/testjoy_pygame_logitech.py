import pygame
from time import sleep
pygame.init()
controller = pygame.joystick.Joystick(0)
controller.init()
buttons = {'A':0,   'B':0,   'X':0,    'Y':0,   
           'L1':0,  'L2':0,  'back':0, 'start':0, 'logi':0,   
           'p_left':0,  'p_right':0,
           'axis0':0.,'axis1':0.,'axis2':0.,'axis3':0.,'axis4':0.,'axis5':0.,
           'hatx':0  ,'haty':0}
axiss = [0.,0.,0.,0.,0.,0.]
hats = [0,0]

 
def getJS(name=''):
 
    global buttons
    # retrieve any events ...
    for event in pygame.event.get():                                # Analog Sticks
        if event.type == pygame.JOYAXISMOTION:
            axiss[event.axis] = round(event.value,2)
        elif event.type == pygame.JOYHATMOTION:
            # print(event.dict, event.joy, event.hat, event.value)
            hats = event.value
            buttons['hatx'] = hats[0]
            buttons['haty'] = hats[1]
            # print(f'hatx : {hatx} , haty : {haty}')
        elif event.type == pygame.JOYBUTTONDOWN:                    # When button pressed
            # print(event.dict, event.joy, event.button, 'PRESSED')
            for x,(key,val) in enumerate(buttons.items()):
                if x<11:
                    if controller.get_button(x):buttons[key]=1
        elif event.type == pygame.JOYBUTTONUP:                      # When button released
            # print(event.dict, event.joy, event.button, 'released')
            for x,(key,val) in enumerate(buttons.items()):
                if x<11:
                    if event.button ==x:buttons[key]=0
 
    # to remove element 2 since axis numbers are 0 1 3 4
    buttons['axis0'],buttons['axis1'] ,buttons['axis2'] ,buttons['axis3'] ,buttons['axis4'] ,buttons['axis5'] = [axiss[0],axiss[1],axiss[2],axiss[3],axiss[4],axiss[5]]
    if name == '':
        return buttons
    else:
        return buttons[name]
def main():
    print(getJS()) # To get all valuesx
    # getJS()
    #sleep(0.05)
    # print(getJS('share')) # To get a single value
    sleep(0.05)
 
 
if __name__ == '__main__':
  while True:
    main()