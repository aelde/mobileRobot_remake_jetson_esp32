import Jetson.GPIO as GPIO
import time

# Pin configuration
GPIO.setmode(GPIO.BOARD)
interrupt_pin = 36
counter = 0

GPIO.setup(interrupt_pin, GPIO.IN)

last_state = GPIO.input(interrupt_pin)

# Define interrupt callback function
def my_callback_f(channel):
    global counter
    global last_state
    
    state = GPIO.input(interrupt_pin)
    
    if state != last_state:
        counter += 1
    else:
        pass
        # counter -= 1
        # if counter < 0 : counter = 0
    
    last_state = state
    print("Encoder count:", counter)
    print("Interrupt detected on pin:", interrupt_pin)

def my_callback(channel):
    if GPIO.input(interrupt_pin) == 1:
        my_callback_f(interrupt_pin)
# Add event detection to the GPIO pin
GPIO.add_event_detect(interrupt_pin, GPIO.RISING, callback=my_callback)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on Ctrl+C exit

