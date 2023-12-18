import Jetson.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define pins
pin_a = 35
pin_b = 22

# Set up GPIO pins for input
GPIO.setup(pin_a, GPIO.IN)
GPIO.setup(pin_b, GPIO.IN)

# Initialize variables
counter = 0
last_a_state = GPIO.input(pin_a)

# Function to handle encoder reading
def read_encoder(channel):
    global counter
    global last_a_state
    
    a_state = GPIO.input(pin_a)
    b_state = GPIO.input(pin_b)
    
    if a_state != last_a_state:
        if b_state != a_state:
            counter += 1
        else:
            counter -= 1
    
    last_a_state = a_state
    print("Encoder count:", counter)

# Add event detection for pin_a
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=read_encoder, bouncetime=5)

try:
    while True:
        time.sleep(1)  # Keep the script running
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on Ctrl+C exit