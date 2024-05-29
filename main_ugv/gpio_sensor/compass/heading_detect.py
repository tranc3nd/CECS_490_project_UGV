import time
from threading import Thread
import paho.mqtt.client as mqtt
from gpio_sensor.compass.bmm150_i2c_spi import bmm150_I2C
from collections import deque
from statistics import median
from decimal import Decimal

# Constants
I2C_BUS = 0x01
ADDRESS_3 = 0x13
COMPASS_DATA_TOPIC = "compass_pub"
BROKER_HOSTNAME = "localhost"
PORT = 1883
WEB_PORT = 9001
MEDIAN_FILTER_SIZE = 5  # Number of samples to consider for median calculation

# Initialize sensor
bmm150 = bmm150_I2C(I2C_BUS, ADDRESS_3)

# MQTT Clients Setup
client = mqtt.Client("Clientcompassstat_1883")
client_web = mqtt.Client("Clientcompassstat_9001", transport='websockets')

def on_connect(client, userdata, flags, rc):
    print("Connected successfully." if rc == 0 else f"Failed to connect, return code: {rc}")

client.on_connect = on_connect
client_web.on_connect = on_connect
client.connect(BROKER_HOSTNAME, PORT)
client_web.connect(BROKER_HOSTNAME, WEB_PORT)
client.loop_start()
client_web.loop_start()

def sensor_setup():
    while bmm150.ERROR == bmm150.sensor_init():
        print("Sensor initialization error, retrying in 1 second...")
        time.sleep(1)
    bmm150.set_operation_mode(bmm150.POWERMODE_NORMAL)
    bmm150.set_preset_mode(bmm150.PRESETMODE_HIGHACCURACY)
    bmm150.set_rate(bmm150.RATE_30HZ)
    bmm150.set_measurement_xyz()

def publish_sensor_data():
    sensor_setup()
    degree_history = deque(maxlen=MEDIAN_FILTER_SIZE)
    degree_history_2nd = degree_history

    while True:
        try:
            degree = bmm150.get_compass_degree()
            degree_history.append(degree)
            filtered_degree = median(degree_history) if len(degree_history) == MEDIAN_FILTER_SIZE else degree
            degree_acc = Decimal(filtered_degree)
            degree = [float(degree_acc.quantize(Decimal('0.000001')))]
            client.publish(COMPASS_DATA_TOPIC, str(degree))
            client_web.publish(COMPASS_DATA_TOPIC, str(degree))
            time.sleep(0.01)
        except Exception as e:
            print(f"Error during data publication: {str(e)}")
            break

def run_compass():
    publish_thread = Thread(target=publish_sensor_data, daemon=True)
    publish_thread.start()
    return publish_thread

def stop_compass():
    client.disconnect()
    client_web.disconnect()
    bmm150.set_operation_mode(bmm150.POWERMODE_SLEEP)
    print("Compass stopped.")

if __name__ == "__main__":
    try:
        thread = run_compass()
        thread.join()
    except KeyboardInterrupt:
        stop_compass()
        print("Program terminated by user.")
