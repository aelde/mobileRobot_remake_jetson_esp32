import serial
import json
import time

# Open the serial port (adjust the port and baudrate as needed)
ser = serial.Serial('/dev/ttyUSB0', 115200)

def send_json_and_receive_response(json_data):
    # Send JSON data to ESP32
    formatted_json = json.dumps(json_data)
    ser.write((formatted_json + '\n').encode('utf-8'))

    # Read response from ESP32
    response = ser.readline().decode('utf-8').strip()

    try:
        # Parse the received JSON response
        response_data = json.loads(response)
        return response_data
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON response: {e}")
        return None
L1 = 0
L2 = 0
R1 = 0
R2 = 0
# Example JSON messages to send
# json_messages = [
#     {"L1": L1, "L2": L2, "R1": R1, "R2": R2}
# ]

while True:
    json_messages = [
        {"L1": L1, "L2": L2, "R1": R1, "R2": R2}
    ]
    print(json_messages)
    L1 += 1
    L2 += 1
    R1 += 1
    R2 += 1
    for json_message in json_messages:
        # Send JSON message and receive response
        response = send_json_and_receive_response(json_message)

        # Print the received response
        if response:
            print("Received response:")
            print(json.dumps(response, indent=2))
        else:
            print("No valid response received")

        # Add a delay before sending the next message
        time.sleep(3)

# Close the serial port when done (this part may not be reached in an infinite loop)
ser.close()
