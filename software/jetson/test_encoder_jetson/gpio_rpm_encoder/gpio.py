import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
inPin = 36
in_p1 = 35
in_p2 = 31
in_p3 = 22
# in_p4 = 8
in_p5 = 32

GPIO.setup(inPin,GPIO.IN)
GPIO.setup(in_p1,GPIO.IN)
GPIO.setup(in_p2,GPIO.IN)
GPIO.setup(in_p3,GPIO.IN)
GPIO.setup(in_p5,GPIO.IN)
# GPIO.setup(in_p4,GPIO.IN)
while True:
    x = GPIO.input(inPin)
    x1 = GPIO.input(in_p1)
    x2 = GPIO.input(in_p2)
    x3 = GPIO.input(in_p3)
    x5 = GPIO.input(in_p5)
    # x4 = GPIO.input(in_p4)
    print('36:',x,' 35:',x1,' 31:',x2,' 22:',x3,' 32:',x5)
    print()
    time.sleep(0.5)
