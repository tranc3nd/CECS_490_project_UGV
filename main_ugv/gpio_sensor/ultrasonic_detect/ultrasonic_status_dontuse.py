from gpiozero import DistanceSensor
from threading import Thread, Timer, Semaphore
from queue import Queue
import time
import paho.mqtt.client as mqtt
import concurrent.futures
from queue import Queue

# Define GPIO pins
TRIG_PIN = 17
ECHO_PIN = 27

INTERVAL = 0.5  # Publishing interval in seconds

# Create ultrasonic sensor object
sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN)

# Create Publish connection to websockets
topic = "Ultra_pub"
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
client = mqtt.Client("ClientSonicstat")
clientweb = mqtt.Client("ClientSonicwebstat", transport='websockets')
# client.username_pw_set(username="user_name", password="password") # uncomment if you use password auth
clientweb.on_connect = on_connect
clientweb.connect(broker_hostname, webport)
clientweb.loop_start()
client.on_connect = on_connect
client.connect(broker_hostname, port)
client.loop_start()

publish_semaphore = Semaphore(1)

def sensor_reading():
    while True:
        try:
            distance = sensor.distance * 100  # Convert meters to centimeters
            with Semaphore:
                print(distance)
                ultraS = [distance]
                client.publish(topic, str(distance))
                clientweb.publish(topic, str(f'{round(distance, 2)} cm'))
                time.sleep(1)
        except KeyboardInterrupt:
            None


def start_reading_dist():
    thread = Thread(target=sensor_reading, daemon=True)
    thread.start()

if __name__ == '__main__':
    start_reading_dist()
