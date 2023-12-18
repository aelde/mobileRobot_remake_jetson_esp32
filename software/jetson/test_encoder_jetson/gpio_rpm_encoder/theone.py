import Jetson.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
aa = 36
bb = 32

counter = 0
lasttime = 0
rpm = 0
counter_b = 0

GPIO.setup(aa, GPIO.IN)
GPIO.setup(bb, GPIO.IN)

def my_callback(channel):
    global counter
    counter += 1
def my_callback_b(channel):
    global counter_b
    counter_b += 1
GPIO.add_event_detect(aa, GPIO.RISING, callback=my_callback)
GPIO.add_event_detect(bb, GPIO.RISING, callback=my_callback_b)

def timtime_loop():
    global counter,lasttime,rpm,rpm_b,counter_b
    while True:
        currenttime = time.time()
        if currenttime - lasttime >= 1:
            rpm = (counter*60) / (currenttime-lasttime)
            rpm_b = (counter_b*60) / (currenttime-lasttime)
            print('rpm a:',rpm,' rpm b:',rpm_b,' counta:',counter,' countb:',counter_b)
            counter = 0
            counter_b = 0
            lasttime = currenttime
        time.sleep(0.1)

if __name__ == '__main__':
    try:
        timtime_loop()
    except KeyboardInterrupt: GPIO.cleanup()

