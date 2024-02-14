# #!/usr/bin/env python3 
# import rclpy
# from rclpy.node import Node
# from pappap.srv import Command
# from std_msgs.msg import String

# import serial
# import time
# import serial.tools.list_ports

# ------ pygame setting ------
import sys
import pygame
from pygame.locals import *

pygame.joystick.init()

print(pygame.joystick.get_count())

pygame.init()
pygame.display.set_caption('game base')
screen = pygame.display.set_mode((500, 500), 0, 32)
clock = pygame.time.Clock()

joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    print(joystick.get_name())

my_square = pygame.Rect(50, 50, 50, 50)
my_square_color = 0
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
motion = [0, 0]


count = 0

while True:

    screen.fill((0, 0, 0))

    pygame.draw.rect(screen, colors[my_square_color], my_square)
    if abs(motion[0]) < 0.1:
        motion[0] = 0
    if abs(motion[1]) < 0.1:
        motion[1] = 0
    my_square.x += motion[0] * 10
    my_square.y += motion[1] * 10

    for event in pygame.event.get():
        if event.type == JOYBUTTONDOWN:
            print(event)
            if event.button == 0:
                my_square_color = (my_square_color + 1) % len(colors)
            # speed
            if event.button == 0:
                if count < 62:
                    count += 1
                    print(f'count = {count}')
                else:
                    count = 62
            if event.button == 2:
                if count > 0:
                    count -= 1
                    print(f'count = {count}')
                else: count = 0
                
            

        if event.type == JOYBUTTONUP:
            print(event)
        if event.type == JOYAXISMOTION:
            print(event)
            if event.axis < 2:
                # motion[event.axis] = event.values
                motion[event.axis] = round(event.value, 2)
                print('X_motion', motion[0])
                print('Y_motion1', motion[1])
                if motion[0] == 0 and motion[1] == 0:
                    print('stop')
                elif motion[1] == -1 and motion[0] == 0 :
                    print('go forward')
                elif motion[1] == 1 and motion[0] == 0:
                    print('go backward')
                elif motion[1] == 0 and motion[0] == -1:
                    print('go left')
                elif motion[1] == 0 and motion[0] == 1:
                    print('go right')
                elif motion[1] == -1 and motion[0] == -1:
                    print('go forward left')
                elif motion[1] == -1 and motion[0] == 1:
                    print('go forward right')
                elif motion[1] == 1 and motion[0] == -1:
                    print('go backward left')
                elif motion[1] == 1 and motion[0] == 1:
                    print('go backward right')
                
                
        if event.type == JOYHATMOTION:
            print(event)
        if event.type == JOYDEVICEADDED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            for joystick in joysticks:
                print(joystick.get_name())
        if event.type == JOYDEVICEREMOVED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()
                
        font_name = pygame.font.match_font('arial') # กำหนดชื่อ Font
        font = pygame.font.Font(font_name, 40) # กำหนดขนาด font
        text_surface = font.render("Speed : " + str(count),True,(255,255,255)) # กำหนด Text และ สี
        text_rect = text_surface.get_rect() # แปลง Surface เป็น object
        text_rect.midtop = (350, 0) # ระบุตำแหน่งของ text
        screen.blit(text_surface, text_rect) # เอา Text ใส่ลงใน object ของ Text นั้น

    pygame.display.update()
    clock.tick(60)