
import serial
import paho.mqtt.client as mqtt
from threading import Thread
#from motor.manual_mov import stop
from loguru import logger
import time
import datetime
from gpiozero import Buzzer
from gpiozero import LED


buzzer = Buzzer(18) #GPIO 2
# Define the GPIO pin number
led_pin = 20  #Do not use pin 3 or else compass i2c will fail

# Initialize the LED object with the GPIO pin number
led = LED(led_pin)

# Create Logger
logger_a = logger.bind(task="Moisture")
logger_a.add("../logs/moisture.log", rotation="100 MB", filter=lambda record: record["extra"]["task"] == "Moisture")

global hit_count

# Open serial connection to ESP32
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.5)

# Create Publish connection to websockets
esp_data_topic = "esp_pub"
moist_topic = "Moist_pub"
msg_count = 0
broker_hostname ="localhost"
port = 1883
webport = 9001
def on_connect(client, userdata, flags, return_code):
    if return_code == 0:
        #print("connected")
        None
    else:
        print("could not connect, return code:", return_code)
client = mqtt.Client("Clientgpsstat")
client_web = mqtt.Client("Clientesp32stat", transport='websockets')
# client.username_pw_set(username="user_name", password="password") # uncomment if you use password auth
client.on_connect = on_connect
client.connect(broker_hostname, port)
client.loop_start()
client_web.on_connect = on_connect
client_web.connect(broker_hostname, webport)
client_web.loop_start()


# Initialize variables to store data
#latitude = None
#longitude = None
siv = None
accuracy = None
moisture = None


def read_esp_data():
    global latitude, longitude
    while True:
        # Read incoming data
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print("Received message:", received_data)
            # Check if received data contains all required fields
            if all(field in received_data for field in ["Lat (deg):", "Lon (deg):", "SIV:", "Accuracy (m):", "Moisture:"]):
                latitude = float(received_data.split(": ")[1].split(",")[0])
                longitude = float(received_data.split(": ")[2].split(",")[0])
                siv = int(received_data.split(": ")[3].split(",")[0])
                accuracy = float(received_data.split(": ")[4].split(",")[0])
                moisture = int(received_data.split(": ")[5])
                read_esp(latitude, longitude, siv, accuracy, moisture)
                read_moist(moisture)

def read_esp(latitude, longitude, siv, accuracy, moisture):
    readings = [ latitude, longitude, siv, accuracy, moisture]
    client_web.publish(esp_data_topic, str(readings))
    client.publish(esp_data_topic, str(readings))

def read_moist(moisture):
    log = ""
    hit_count = 0
    #if moisture > 3800:
    if moisture < 2000:
        hit_count += 1
        buzzer.on()
        if hit_count == 1:
            led.on()
            time.sleep(3)
            buzzer.off()
            led.off()
            read_out = "Water detected!"
            logger_a.critical("Water detected. UGV has been hit. Stopping UGV... | " + "Lat:" + str(latitude) + " Lon:" + str(longitude))
            log = "Time:" + str(datetime.datetime.now()) + " | Water detected. UGV has been hit. Stopping UGV... | " + "Lat:" + str(latitude) + " Lon:" + str(longitude)
    else:
        read_out = "No water detected."

    readings = [ read_out, hit_count, moisture, log ]
    client_web.publish(moist_topic, str(readings))
    client.publish(moist_topic, str(readings))


    
def run_esp():
    thread = Thread(target=read_esp_data, daemon=True)
    thread.start()

if __name__ == '__main__':
    try:
        run_esp()
    except:
        #None
        ser.close()