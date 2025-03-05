import serial
import time

# Open Serial connection (Make sure COM7 is correct)
ser = serial.Serial('COM7', 115200, timeout=1)
time.sleep(2)  # Allow connection setup

# Test sending commands
for command in ['R', 'Y', 'N']:
    ser.write(command.encode())  # Send command to ESP32
    print(f"Sent: {command}")
    time.sleep(2)  # Small delay to observe output
