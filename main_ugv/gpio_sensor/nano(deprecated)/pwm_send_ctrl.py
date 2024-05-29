'''
from serial import Serial

# Setup connection with Nano
ser = Serial('/dev/ttyUSB0',9600, timeout=0.5)

def sendMov(direction):
    ser.write(str.encode(direction))
'''