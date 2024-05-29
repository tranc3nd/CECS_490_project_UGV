from signal import pause
import warnings
warnings.simplefilter('ignore')
from gpiozero import Motor, PWMOutputDevice
from time import sleep
from loguru import logger
from gpio_sensor.motor import speed_control_3
import json
import paho.mqtt.client as mqtt

# Create logger
logger_b = logger.bind(task="Movement")
logger_b.add("../logs/movement.log", rotation="100 MB", filter=lambda record: record["extra"]["task"] == "Movement")

# Pub Sub
broker_hostname ="localhost"
port = 1883
topic1 = "Ultra_pub"

MQTT_TOPICS = [(topic1, 2)]

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPICS)

def on_message(client, userdata, msg):
    global read_ultrasonic, us_bool_check
    us_bool_check = False

    if msg.topic == topic1:
        payload_str = msg.payload.decode("utf-8")  # Decode byte string to UTF-8 string
        data = json.loads(payload_str)  # Parse JSON string to Python dictionary
        read_ultrasonic = int(data)
        if ultra_sonic_result(read_ultrasonic):
            us_bool_check = True
        else:
            us_bool_check = False
        

def initialize_ultra_sonic_feed():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_hostname, port)
    client.loop_start()
    

def ultra_sonic_result(ultrasonic):
    distance = read_ultrasonic <= 30
    #print("Ultrasonic:" , distance)
    return distance

def setSpeed(setSpeedMPH):
    global speed
    speed = float(setSpeedMPH)
    print("SpeedSet:" ,speed)


def move_forward():
    direction_input = "B"
    speed_control_3.left_controller(direction_input, speed=speed)
    speed_control_3.right_controller(direction_input, speed=speed)

def move_backward():
    direction_input = "BW"
    speed_control_3.left_controller(direction_input, speed=speed)
    speed_control_3.right_controller(direction_input, speed=speed)

def turn_left():
    # Turning left
    direction_input = "L" 
    speed_control_3.right_controller(direction_input, speed=speed)
    speed_control_3.left_controller(direction_input, speed=speed)

def turn_right():
    # Turning right
    direction_input = "R"
    speed_control_3.right_controller(direction_input, speed=speed)
    speed_control_3.left_controller(direction_input, speed=speed)

def stop():
    # Stop
    speed_control_3.left_controller("S", 0)
    speed_control_3.right_controller("S", 0)

def manual_dir(direction): 
    if us_bool_check == False:
        if direction == "200":
            move_forward()
        elif direction == "201":
            move_backward()
        elif direction == "202":
            turn_left()
        elif direction == "203":
            turn_right()
        elif direction == "204":
            stop()
    elif us_bool_check == True:
        stop()
    else:
        stop()
