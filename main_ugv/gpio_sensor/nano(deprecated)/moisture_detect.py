import serial
import time
import concurrent.futures
from threading import Thread
from gpiozero import Buzzer
from gpiozero import LED
from pathlib import Path
from sys import path
from loguru import logger
import paho.mqtt.client as mqtt

# Create logger file
logger_a = logger.bind(task="Moisture")
logger_a.add("../logs/moisture.log", rotation="500 MB", filter=lambda record: record["extra"]["task"] == "Moisture")

object_dir = str(Path.cwd().parent)
path.insert(0, object_dir)
from gpio_sensor.motor.manual_mov import stop
#from motor.manual_mov import stop

# Open serial port (replace '/dev/ttyUSB0' with your serial device)
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Allow some time for the serial connection to initialize
#time.sleep(2)

buzzer = Buzzer(2) #GPIO 2
# Define the GPIO pin number
led_pin = 3

# Initialize the LED object with the GPIO pin number
led = LED(led_pin)

global hit_count


# Create Publish connection to websockets
topic = "Moist_pub"
msg_count = 0
broker_hostname ="localhost"
port = 9001 
def on_connect(client, userdata, flags, return_code):
    if return_code == 0:
        #print("connected")
        None
    else:
        print("could not connect, return code:", return_code)
client = mqtt.Client("Clientwifistat", transport='websockets')
# client.username_pw_set(username="user_name", password="password") # uncomment if you use password auth
client.on_connect = on_connect
client.connect(broker_hostname, port)
client.loop_start()

def read_moisture():
    while True:
        hit_count = 0
        #while True:  # Continues indefinitely until the program is terminated
            #value = adc.value * 100  # Convert to percentage
            #if 20 < value < 50:
        led.off()
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            #print(line)
            if int(line) < 49:
                hit_count += 1
                buzzer.on()
                if hit_count == 1:
                    led.on()
                    stop()
                    #time.sleep(1)
                    #buzzer.off()
                    read_out = "Water detected!"
                    logger_a.critical("Water detected. UGV has been hit. Stopping UGV...")
                    # Turn on the LED

                #print("LED is ON")
                #time.sleep(2)  # Delay to simulate processing; adjust as needed
            else:
                read_out = "No water detected."
            readings = [ read_out, hit_count, line ]
            #print(readings)
            #time.sleep(0.2)
            client.publish(topic, str(readings))



def start_reading_moist():
    thread = Thread(target=read_moisture, daemon=True)
    thread.start()

if __name__ == '__main__':
    while True:
        start_reading_moist()