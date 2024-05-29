'''
import sys
import os
import time
import RPi.GPIO as GPIO  # Import GPIO library

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_bmm150 import *

RASPBERRY_PIN_CS = 27  # Chip selection pin when SPI is selected, use BCM coding method, the number is 27, corresponding to pin GPIO2
I2C_BUS = 0x01  # default use I2C1
ADDRESS_3 = 0x13  # (CSB:1 SDO:1) default i2c address
bmm150 = DFRobot_bmm150_I2C(I2C_BUS, ADDRESS_3)

# Variable to store the initial compass degree
initial_degree = None

# Set up GPIO for interrupt pin
GPIO.setmode(GPIO.BCM)
INTERRUPT_PIN = 17  # Example GPIO pin for interrupt, you can change it according to your setup
GPIO.setup(INTERRUPT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def setup():
    global initial_degree
    while bmm150.ERROR == bmm150.sensor_init():
        print("sensor init error, please check connect")
        time.sleep(1)

    bmm150.set_operation_mode(bmm150.POWERMODE_NORMAL)
    bmm150.set_preset_mode(bmm150.PRESETMODE_HIGHACCURACY)
    bmm150.set_rate(bmm150.RATE_10HZ)
    bmm150.set_measurement_xyz()

    # Get the initial compass degree
    initial_degree = bmm150.get_compass_degree()

    # Configure interrupt handler
    GPIO.add_event_detect(INTERRUPT_PIN, GPIO.BOTH, callback=interrupt_handler, bouncetime=200)

def interrupt_handler(channel):
    global initial_degree
    degree = bmm150.get_compass_degree()

    # Calculate the offset from the initial degree
    if initial_degree is not None:
        offset_degree = degree - initial_degree
        print("Offset from initial state: %.2f degrees" % offset_degree)

        # Check if offset crosses 90 degrees in either direction
        if abs(offset_degree) >= 90:
            print("Offset reached 90 degrees in either direction!")
    else:
        print("Initial degree not yet captured.")

def loop():
    pass  # Since the interrupt handler will handle updates, this loop can be empty

if __name__ == "__main__":
    setup()
    try:
        while True:
            time.sleep(1)  # Keep the program running
    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO on program exit

        '''
