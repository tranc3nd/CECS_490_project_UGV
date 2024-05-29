import time
import paho.mqtt.client as mqtt
from gpiozero import DistanceSensor
from threading import Thread, Event, Semaphore
from queue import Queue

# Configuration constants
TRIG_PIN = 17
ECHO_PIN = 27
BROKER_HOSTNAME = "localhost"
PORT = 1883
WEB_PORT = 9001
TOPIC = "Ultra_pub"
PUBLISH_INTERVAL = 0.5  # Interval to check sensor and publish if changed
MAX_CONCURRENT_PUBLISHES = 1  # Max concurrent MQTT publishes

q = Queue()

# Initialize sensor
sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN)

# Semaphore to control access to the MQTT publish function
publish_semaphore = Semaphore(MAX_CONCURRENT_PUBLISHES)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully.")
    else:
        print(f"Could not connect, return code: {rc}")

def setup_mqtt_client(name, port, transport=None):
    client = mqtt.Client(name, transport=transport) if transport else mqtt.Client(name)
    client.on_connect = on_connect
    client.connect(BROKER_HOSTNAME, port)
    client.loop_start()
    return client

client = setup_mqtt_client("ClientSonicstat", PORT)
client_web = setup_mqtt_client("ClientSonicwebstat", WEB_PORT, transport='websockets')

def publish_sensor_data(stop_event):
    """Publish sensor data to MQTT broker and check for stop event."""
    last_distance = None
    try:
        while not stop_event.is_set():
            distance = round(sensor.distance * 100, 2)  # Convert meters to centimeters
            if distance != last_distance:
                with publish_semaphore:  # Ensure only one thread can enter this block at a time
                    #print(f"Distance: {distance} cm")  # Print the current distance
                    #q.put_nowait(distance)
                    client.publish(TOPIC, str(distance))
                    client_web.publish(TOPIC, str(distance))
                last_distance = distance
            stop_event.wait(PUBLISH_INTERVAL)  # Efficient wait
    finally:
        client.loop_stop()
        client_web.loop_stop()

def start_reading_dist():
    stop_event = Event()
    thread = Thread(target=publish_sensor_data, args=(stop_event,), daemon=True)
    thread.start()

if __name__ == "__main__":
    start_reading_dist()
