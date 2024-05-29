from gpiozero import DistanceSensor
from threading import Thread
from queue import Queue
import time
import paho.mqtt.client as mqtt

# GPIO numbers
TRIG = 17
ECHO = 27

distSensor = DistanceSensor(echo=ECHO, trigger=TRIG)
q = Queue()


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


def sensor_reading():
    while True:
        distance_cm = distSensor.distance * 100  # Convert to cm
        if distance_cm < 50: 
            #q.put_nowait(f'{round(distance_cm, 2)} cm')
            ultraS = [distance_cm]
            client.publish(topic, ultraS)
            clientweb.publish(topic, str(f'{round(distance_cm, 2)} cm'))
        '''
        # Dynamically adjust sleep based on distance
        if distance_cm < 10:
            time.sleep(0.05)  # Faster response for very close objects
        elif distance_cm < 30:
            time.sleep(0.1)   # Moderate delay for objects within an intermediate range
        else:
            time.sleep(0.2)   # Relatively longer delay for distant or no objects
        '''

def start_reading_dist():
    Thread(target=sensor_reading, daemon=True).start()

if __name__ == '__main__':
    queue = start_reading_dist()
