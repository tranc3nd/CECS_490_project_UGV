
import json
from haversine import haversine, Unit
import pyproj 
from pyproj import Geod
from gpiozero import Motor, PWMOutputDevice, Button
from time import sleep
import time
import paho.mqtt.client as mqtt
from flask import Flask, request, jsonify

# Motor and PWM setup
motor_R = Motor(forward=20, backward=19)
motor_L = Motor(forward=12, backward=13)
pwm_R = PWMOutputDevice(pin=21, frequency=100)
pwm_L = PWMOutputDevice(pin=6, frequency=100)

# Encoder setup using gpiozero Button to detect rising edges
encoder_R = Button(22)
encoder_L = Button(24)

# Encoder counters
counter_R = 0
counter_L = 0

# Constants for calculation
pulses_per_revolution = 616  # Adjust this to your encoder's specification
wheel_circumference = 15.7 # centimeters (adjust to your wheel's circumference)

# Increment counter on encoder pulse
def increment_counter_R():
    global counter_R
    counter_R += 1

def increment_counter_L():
    global counter_L
    counter_L += 1

# Attach the increment function to the encoder's "when_pressed" event
encoder_R.when_pressed = increment_counter_R
encoder_L.when_pressed = increment_counter_L

def calculate_distance(counter, pulses_per_revolution, wheel_circumference):
    revolutions = counter / pulses_per_revolution
    distance = revolutions * wheel_circumference  # Distance in centimeters
    return distance


def reset_counters():
    global counter_R, counter_L
    counter_R = 0
    counter_L = 0

geodesic = pyproj.Geod(ellps='WGS84') #common GPS model

latitude = None
longitude = None
siv = None
accuracy = None

#Pub Sub
broker_hostname ="localhost"
port = 1883
topic = "Gps_pub"
#topic2 = "Moist_pub"

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic)
    #client.subscribe(topic2)

def on_message(client, userdata, msg):
    global siv, accuracy, current_location 
    payload_str = msg.payload.decode("utf-8")  # Decode byte string to UTF-8 string
    data = json.loads(payload_str)  # Parse JSON string to Python dictionary
    current_location = (float(data[0]), float(data[1])) #lat,long from gps
    print("currentloc:", current_location)
    siv = int(data[2])  # Convert siv to int
    accuracy = float(data[3])  # Convert accuracy to float

def live_gps_data(target_location):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_hostname, port)
    client.loop_start()
    time.sleep(1)
    receive_data(target_location)

#Receives Target Location
def receive_data(target_location):
    #global distance_goal, fwd_azimuth
    global on_track
    print("targetloc:", target_location)
    #Calculate the distance
    distance_goal = haversine(target_location, current_location, unit=Unit.METERS) #receive data to go here 
    print("Your distance to travel is: ", distance_goal, "meters")
    inter=geodesic.npts(current_location[1], current_location[0], target_location[1], target_location[0], 20)
    coordinates_array = [(point[1], point[0]) for point in inter]
    print("Intermediate points: ")
    for point in coordinates_array:
        print(f"Latitude: {point[0]}, Longitude: {point[1]}")
    
    tolerance = 0.000001 #okay if inaccurate by this much
    on_track = False  # Reset on_track flag before comparison
    
    if (distance_goal is not None): #user has selected target on google map
        
        for index, point in enumerate(coordinates_array, start=1): #compare all 20 lat long pairs with current location lat long pairs
            time.sleep(1.5) #allow time in between comparisons for current location to update CONCERN: current locaiton not being updated quick enough
            print(f"\nChecking Point {index}: Latitude: {point[0]}, Longitude: {point[1]}")
            print(f"Comparing with current location: Latitude: {current_location[0]}, Longitude: {current_location[1]}")
            # Compare latitude and longitude within tolerance
            if abs(point[0] - current_location[0]) <= tolerance and abs(point[1] - current_location[1]) <= tolerance:
                print("on track ish", point)
                on_track = True
                print("Continue Forward")
                motor_R.forward()
                motor_L.forward()
                pwm_R.value = 0.5
                pwm_L.value = 0.5
                distance_a = calculate_distance(counter_R, pulses_per_revolution, wheel_circumference)
                distance_b = calculate_distance(counter_L, pulses_per_revolution, wheel_circumference)
                print(f"Distance traveled: Motor R: {distance_a}cm, Motor L: {distance_b}cm")
                sleep(0.4)
                if (distance_a > distance_goal and distance_b > distance_goal): 
                    motor_R.stop()
                    motor_L.stop()
                    break
            else:
                #need more handling, CONCERN
                on_track = False
                print("veering off")



'''
MOTOR NAVI 
3 CLICKS TO GOOGLE MAPS, AND CAPTURE 
NEW POWER SYSTEM (BATTERY ORDERED, NEED NEW BUCK MAYBE)
AI ON THE CAMERA
NEW BODY 


'''

