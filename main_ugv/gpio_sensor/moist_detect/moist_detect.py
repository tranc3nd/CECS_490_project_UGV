# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola, Christian Valdez
# License: Public Domain
import time

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
Moisture_Level = 0 #define int for tracking moisture
Moisture_Threshold = 600 #output value when water present
Hit_Detect = 0 #0 not hit, 1 hit

print('Reading MCP3008 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
print('-' * 57)
# Main program loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0]*8
    for i in range(8):
        # The read_adc function will get the value of the specified channel (0-7).
        values[i] = mcp.read_adc(i)
    Moisture_Level = mcp.read_adc(7) #sensor on channel 8
    # Print the ADC values.
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    if (Moisture_Level <= Moisture_Threshold):
        print("Water Detected! ")
        Hit_Detect = 1 #value we need to GET with API 
    # Pause for half a second.
    time.sleep(2)

def getHitStatus():
    return Hit_Detect