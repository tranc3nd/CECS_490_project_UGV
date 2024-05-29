import json
from haversine import haversine, Unit
import pyproj 
from pyproj import Geod
from gpiozero import Motor, PWMOutputDevice, Button, Device
from time import sleep, time
#import time
import paho.mqtt.client as mqtt
from flask import Flask, request, jsonify
import math

# Motor and PWM setup
motor_R = (20, 19)
motor_L = (13, 12)
pwm_R = PWMOutputDevice(pin=21, frequency=1000)
pwm_L = PWMOutputDevice(pin=6, frequency=1000)

# Encoder setup using gpiozero Button to detect rising edges
encoder_Ra = Button(24)
encoder_Rb = Button(23)
encoder_La = Button(22)
encoder_Lb = Button(25)

# Speed calculation variables
encoder_ticks_R = 0
encoder_ticks_L = 0
last_time_checked = time()
speed_R = 0.0  # Speed in mph
speed_L = 0.0  # Speed in mph

# Encoder Constants 
WHEEL_CIRCUMFERENCE = 15.7  # in meters (example value)
TICKS_PER_REVOLUTION = 616   # adjust with your specific encoder
MPS_TO_MPH = 2.23694  # Conversion factor from meters per second to miles per hour

# General Constants
latitude = None
longitude = None
siv = None
accuracy = None

# Pub Sub
broker_hostname ="localhost"
port = 1883
topic1 = "esp_pub"
topic2 = "compass_pub"

# TOPICS
MQTT_TOPICS = [(topic1, 2), (topic2, 2)]

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPICS)

def on_message(client, userdata, msg):
    global siv, accuracy, current_location, moisture_lvl, read_current_heading

    if msg.topic == topic1:
        payload_str = msg.payload.decode("utf-8")  # Decode byte string to UTF-8 string
        data = json.loads(payload_str)  # Parse JSON string to Python dictionary
        current_location = (float(data[0]), float(data[1])) #lat,long from gps
        print("currentloc:", current_location)
        siv = int(data[2])  # Convert siv to int
        accuracy = float(data[3])  # Convert accuracy to float
        moisture_lvl = int(data[4])
        print("Moisture Level:" + str(moisture_lvl))
    elif msg.topic == topic2:
        payload_str = msg.payload.decode("utf-8")  # Decode byte string to UTF-8 string
        data = json.loads(payload_str)  # Parse JSON string to Python dictionary
        read_current_heading = int(data[0])
        #print("reading compass:" + str(read_current_heading))

def live_gps_data(target_location):
    navigate(target_location)

# Calculate bearing
def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the compass bearing from the first point (lat1, lon1) to the second point (lat2, lon2).
    
    Args:
    lat1 (float): Latitude of the first point in degrees.
    lon1 (float): Longitude of the first point in degrees.
    lat2 (float): Latitude of the second point in degrees.
    lon2 (float): Longitude of the second point in degrees.
    
    Returns:
    float: The compass direction as degrees.
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Calculate change in coordinates
    delta_lon = lon2 - lon1

    # Trigonometric calculations
    y = math.sin(delta_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

    # Compute initial bearing in radians and convert to degrees
    bearing = math.atan2(y, x)
    bearing_degrees = math.degrees(bearing)

    # Normalize bearing to 0-360 degrees
    bearing_degrees = (bearing_degrees + 360) % 360
    #bearing_degrees = (bearing_degrees + 360)

    # Compass bearings
    compass_bearings = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

    # Determine the compass bearing index
    index = int((bearing_degrees + 22.5) // 45) % 8

    #return compass_bearings[index]
    print("Bearing Deg: " + str(bearing_degrees))
    return bearing_degrees

def read_moist(moisture):
    detect_moist = moisture > 3800
    return detect_moist

def initialize_cur_loc():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_hostname, port)
    client.loop_start()
    #print("read compass:" + str(read_current_heading))
    #time.sleep(0.5)

class UGVControls:
    def __init__(self, left_motor_pin1, left_motor_pin2, right_motor_pin1, right_motor_pin2):
        self.left_motor = Motor(forward=left_motor_pin1, backward=left_motor_pin2)
        self.right_motor = Motor(forward=right_motor_pin1, backward=right_motor_pin2)
        self.heading = 0
        self.speed = 0  # Speed in percentage (0-1)

    def set_heading(self, bearing):
        global pivot_turn

        print("Current Degree:" + str(read_current_heading))
        print("Target Degree:" + str(bearing))

        # Calculate the shortest pivot turn angle, normalizing between -180 to 180 degrees
        pivot_turn = (bearing - read_current_heading + 180) % 360 - 180
        
        # Proportional control damping factor (tune this based on your robot's performance)
        kp = 0.1
        adjusted_speed = max(min(abs(pivot_turn) * kp, self.speed), 0)  # Clamping the speed between 0 and max speed
        
        if abs(pivot_turn) > 10:
            if pivot_turn > 0:
                # Turning right
                self.left_motor.forward(speed=adjusted_speed)
                self.right_motor.backward(speed=adjusted_speed)
                print("turning right with adjusted speed:", adjusted_speed)
            else:
                # Turning left
                self.left_motor.backward(speed=adjusted_speed)
                self.right_motor.forward(speed=adjusted_speed)
                print("turning left with adjusted speed:", adjusted_speed)
        else:
            # Moving forward
            self.left_motor.forward(speed=self.speed)
            self.right_motor.forward(speed=self.speed)
            print("moving forward with full speed")

    def set_speed(self, speed):
        self.speed = speed
        self.left_motor.forward(speed=speed)
        self.right_motor.forward(speed=speed)
        pwm_R.value = speed
        pwm_L.value = speed
        print("Default Speed: " + str(speed))

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()
        pwm_R.value = 0
        pwm_L.value = 0

def move_ugv_to_target(start_lat, start_lon, target_lat, target_lon, ugv_controls):
    current_lat, current_lon = start_lat, start_lon
    update_interval = 0.1  # Update interval in seconds
    while True:
        calculate_speed()
        bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        ugv_controls.set_speed(1)
        ugv_controls.set_heading(int(bearing))

        print("Pivot Turn:" + str(pivot_turn))
        if abs(pivot_turn) < 10 and abs(pivot_turn) > 0:
            distance = 0.05  # Assume a small constant distance per time step
            current_lat, current_lon = calculate_new_position(current_lat, current_lon, bearing, distance)
        
        if read_moist(moisture_lvl):
            ugv_controls.stop()
            print("UGV has been hit. Stopping...")
            break
        
        if is_target_reached(current_lat, current_lon, target_lat, target_lon, threshold=0.2) == True:
            ugv_controls.stop()
            print("Target reached.")
            break
            
        sleep(update_interval)

def is_target_reached(current_lat, current_lon, target_lat, target_lon, threshold):
    current_pos = (current_lat, current_lon)
    target_pos = (target_lat, target_lon)
    cur_distance = haversine(target_pos, current_pos, unit=Unit.METERS) 
    print("Current Pos:" + str(current_pos) + " Target Pos:" + str(target_pos) + " Threshold:" + str(threshold))
    print("Current Distance:" + str(cur_distance))
    target_reached = cur_distance < threshold
    print("Target Reached:" + str(target_reached))
    return target_reached


def calculate_new_position(lat, lon, bearing, distance):
    """
    Calculate new position given a start point, bearing and distance.
    
    Args:
    lat (float): Latitude of the starting point.
    lon (float): Longitude of the starting point.
    bearing (float): Bearing in degrees.
    distance (float): Distance to move in kilometers.
    
    Returns:
    tuple: A tuple containing the new latitude and longitude.
    """
    R = 6371000  # Earth's radius in meters
    bearing = math.radians(bearing)

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                     math.cos(lat1) * math.sin(distance / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat1),
                             math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))
    
    print(math.degrees(lat2))
    print(math.degrees(lon2))

    return math.degrees(lat2), math.degrees(lon2)

# Receives Target Location
def navigate(target_locations):
    # Looping through the dictionary and unpacking the latitude and longitude from each tuple
    for location, (latitude, longitude) in target_locations.items():
        print("Target Location:" + str(location))
        ugv_controls = UGVControls(*motor_L, *motor_R)
        move_ugv_to_target(current_location[0], current_location[1], latitude, longitude, ugv_controls)
        print("Target location " + str(location) + " has been reached...")
        sleep(0.5)

# Calculate Speed functions
def encoder_callback_R():
    global encoder_ticks_R
    encoder_ticks_R += 1

def encoder_callback_L():
    global encoder_ticks_L
    encoder_ticks_L += 1

encoder_Ra.when_pressed = encoder_callback_R
encoder_La.when_pressed = encoder_callback_L

def calculate_speed():
    global encoder_ticks_R, encoder_ticks_L, last_time_checked, speed_R, speed_L

    current_time = time()
    elapsed_time = current_time - last_time_checked

    # Distance traveled = number of ticks / ticks per revolution * circumference
    distance_R = (encoder_ticks_R / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE
    distance_L = (encoder_ticks_L / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE

    # Speed in m/s
    speed_R_mps = distance_R / elapsed_time
    speed_L_mps = distance_L / elapsed_time

    # Convert speed to mph
    speed_R = speed_R_mps * MPS_TO_MPH
    speed_L = speed_L_mps * MPS_TO_MPH

    # Reset ticks and update last checked time
    encoder_ticks_R = 0
    encoder_ticks_L = 0
    last_time_checked = current_time

    # Log speeds for debugging (or use these values in your control logic)
    print(f"Right wheel speed: {speed_R} mph, Left wheel speed: {speed_L} mph")