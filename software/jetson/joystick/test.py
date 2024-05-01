def map_value(input_value, in_min, in_max, out_min, out_max):
    # Map the input value from the input range to the output range
    return (input_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

input_value = -1  # Example input value, adjust as needed
mapped_value = map_value(input_value, -1, 1, 1, 60)

print(mapped_value)  # Output the mapped value
