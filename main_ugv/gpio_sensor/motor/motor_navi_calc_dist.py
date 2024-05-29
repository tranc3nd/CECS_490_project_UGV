import json
from haversine import haversine, Unit
import pyproj 
from pyproj import Geod
#from gpiozero import Motor, PWMOutputDevice, Button
from time import sleep, time
import paho.mqtt.client as mqtt
from flask import Flask, request, jsonify
import math
from decimal import Decimal
from loguru import logger
#from speed_control_3 import left_Motor, right_Motor, both_Motor, stop_motors # NEW
#from gpio_sensor.motor.speed_control_3 import right_turn, left_turn, moving_forward, stop_motors # NEW
from gpio_sensor.motor import speed_control_3

from gpiozero import Buzzer
from gpiozero import LED

#buzzer = Buzzer(6) #GPIO 2
# Define the GPIO pin number
#led_pin = 5  #Do not use pin 3 or else compass i2c will fail

# Initialize the LED object with the GPIO pin number
#led = LED(led_pin)


# Create Logger
logger_a = logger.bind(task="MotorNavi")
logger_a.add("../logs/motor_navi.log", rotation="500 MB", filter=lambda record: record["extra"]["task"] == "MotorNavi")


# Motor and PWM setup
#motor_R = (20, 19)
#motor_L = (13, 12)
#pwm_R = PWMOutputDevice(pin=21, frequency=100)
#pwm_L = PWMOutputDevice(pin=6, frequency=100)

# Encoder setup using gpiozero Button to detect rising edges
#encoder_Ra = Button(24)
#encoder_Rb = Button(23)
#encoder_La = Button(22)
#encoder_Lb = Button(25)

#counter_Ra = 0 #NEW
#counter_La = 0 #NEW

# General Constants
latitude = None
longitude = None
siv = None
accuracy = None
#speedMph = 0.6


# Pub Sub
broker_hostname ="localhost"
port = 1883
topic1 = "esp_pub"
topic2 = "compass_pub"
topic3 = "Ultra_pub"

# TOPICS
MQTT_TOPICS = [(topic1, 2), (topic2, 2)]

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPICS)

def on_message(client, userdata, msg):
    global siv, accuracy, current_location, moisture_lvl, read_current_heading, average_heading, ultrasonic

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
    elif msg.topic == topic3:
        payload_str = msg.payload.decode("utf-8")  # Decode byte string to UTF-8 string
        data = json.loads(payload_str)  # Parse JSON string to Python dictionary
        ultrasonic = int(data[0])

        #print("reading compass:" + str(read_current_heading))

def live_gps_data(target_location, challenge):
    navigate(target_location, challenge)

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

    #return compass_bearings[index]
    print("Bearing Deg: " + str(bearing_degrees))
    return bearing_degrees

def read_moist(moisture):
    detect_moist = moisture < 2000
    return detect_moist

def initialize_cur_loc():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_hostname, port)
    client.loop_start()

def get_updating_degree():
    return read_current_heading

class UGVControls:
    #def __init__(self, left_motor_pin1, left_motor_pin2, right_motor_pin1, right_motor_pin2):
    def __init__(self):
        #self.left_motor = Motor(forward=left_motor_pin1, backward=left_motor_pin2)
        #self.right_motor = Motor(forward=right_motor_pin1, backward=right_motor_pin2)
        #self.heading = 0
        #self.speed = 0 
        self.speed = 0
        self.integral_error = 0  # Initialize integral error for PID controller

    def set_heading(self, current_degree, target_heading, tolerance):
        global pivot_turn, direction_input
        # Log and print the current degree
        logger_a.info(f"Current Degree: {current_degree}")
        print(f"Current Degree: {current_degree}")

        # Calculate the shortest pivot turn angle, normalizing between -180 to 180 degrees
        pivot_turn = (target_heading - current_degree + 180) % 360 - 180
        logger_a.info(f"Pivot Turn: {pivot_turn}")
        print(f"Pivot Turn: {pivot_turn}")
        print("Set Heading Current Speed:" , self.speed)

        # Update integral error
        self.integral_error += pivot_turn

        # Proportional and integral gains for PID controller
        kp = 0.1 # Proportional gain
        ki = 0.01  # Integral gain

        # Calculate adjusted speed with PI components
        #adjusted_speed = max(min(abs(pivot_turn) * kp + abs(self.integral_error) * ki, self.speed), 0)
        adjusted_speed = self.speed
        print("Adjusted Speed:" , adjusted_speed)
        # Determine motor actions based on the pivot turn and tolerance
        if abs(pivot_turn) >= tolerance:
            if pivot_turn > 0:
                direction_input = "R"
                print("Pivot:", pivot_turn)
                speed_control_3.right_controller(direction_input, adjusted_speed)
                speed_control_3.left_controller(direction_input, adjusted_speed)
                logger_a.info(f"Turning right with adjusted speed: {adjusted_speed}")
                print(f"Turning right with adjusted speed: {adjusted_speed}")
            else:
                direction_input = "L" 
                print("Pivot:", pivot_turn)
                speed_control_3.right_controller(direction_input, adjusted_speed)
                speed_control_3.left_controller(direction_input, adjusted_speed)
                logger_a.info(f"Turning left with adjusted speed: {adjusted_speed}")
                print(f"Turning left with adjusted speed: {adjusted_speed}")  
        else:
            direction_input = "B"
            print("Pivot:", pivot_turn)
            speed_control_3.left_controller(direction_input, self.speed)
            speed_control_3.right_controller(direction_input, self.speed)
            print("Forward Speed: " + str(self.speed))
            logger_a.info("Forward Speed:" + str(self.speed))
            #logger_a.info(f"Moving forward with speed: {self.speed}")
            #print(f"Moving forward with speed: {self.speed}")
            
            

        # Set PWM values depending on pivot turn size
        #pwm_L.value = pwm_R.value = adjusted_speed if abs(pivot_turn) > tolerance else self.speed


    def set_speed(self, speed):
        self.speed = speed
        #speed_control_3.setup()
        direction_input = "B"
        speed_control_3.left_controller(direction_input, self.speed)
        speed_control_3.right_controller(direction_input, self.speed)
        print("Default Speed: " + str(self.speed))
        logger_a.info("Default Speed:" + str(self.speed))

    def stop(self):
        #self.left_motor.stop()
        #self.right_motor.stop()
        #self.left_motor.close()
        #self.right_motor.close()
        #motor.stop()
        #motor.close()
        
        speed_control_3.left_controller("S", 0)
        speed_control_3.right_controller("S", 0)
        print("Stop here")
        #pwm_R.value = 0
        #pwm_L.value = 0
        

def move_ugv_to_target(start_lat, start_lon, target_lat, target_lon, ugv_controls, initial_heading):
    setSpeedMPH = 0.47
    current_lat, current_lon = start_lat, start_lon

    #diff_1_lon = calibration_1_lon - current_lon
    #diff_1_lat = calibration_1_lat - current_lat
    #diff_2_lon = calibration_2_lon - current_lon
    #diff_2_lat = calibration_2_lat - current_lat

    #finish_1_lon = target_pos + diff_1_lon 
    #finish_1_lat = target_pos + diff_1_lat 
    #finish_2_lon = target_pos + diff_2_lon 
    #finish_2_lat = target_pos + diff_2_lat 

    #if 

    update_interval = 0.1  # Update interval in seconds
    global prev_time, counter_Ra, counter_La, start_time #NEW
    # Using the initial heading provided and adjusting by turn_degree
    maintain_heading = initial_heading
    print("Maintain Heading:", maintain_heading)
    logger_a.info("Maintain Heading:" + str(maintain_heading))
    start_time = time()

    tolerance = 10 # tolerance is used to determine max off degree before it starts to do a pivot turn
    ugv_controls.set_speed(setSpeedMPH)
    while True:
        #start_time = time() #NEW
        #current_time = time() #NEW
        #elapsed_time = current_time - prev_time #NEW
        #bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        print("TargetLat:" , target_lat)
        print("TargetLon:" , target_lon)
        print("Starting Heading")
        ugv_controls.set_heading(read_current_heading, initial_heading, tolerance)
        
        #distance_R = calculate_distance(counter_Ra, TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE) #NEW
        #distance_L = calculate_distance(counter_La, TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE) #NEW
        #counter_Ra = 0 #NEW
        #counter_La = 0 #NEW
        #distance_avg = ((distance_R+distance_L/2)/100) #NEW
        #print("Distance traveled in meters:", distance_avg) #NEW

        # Sleep until 0.1 seconds have passed since the start of the loop
        '''
        time_to_sleep = 0.1 - (time() - start_time)
        if time_to_sleep > 0:
            sleep(time_to_sleep)
        # If one second has passed, reset variables
        if elapsed_time >= 1.0:
            prev_time = current_time
        '''

        #print("Pivot Turn:" + str(pivot_turn))
        if abs(pivot_turn) < tolerance and abs(pivot_turn) > 0:
            #distance = 0.02101088  # Assume a small constant distance per time step, meters
            #current_lat, current_lon = calculate_new_position(current_lat, current_lon, maintain_heading, distance)
            pass

        
        if read_moist(moisture_lvl):
            ugv_controls.stop()
            print("UGV has been hit. Stopping...")
            logger_a.info("UGV has been hit. Stopping...")
            break
        
        #threshold=0.000009
        if is_target_reached(current_lat, current_lon, target_lat, target_lon, setSpeedMPH, threshold=0.5) == True:
            ugv_controls.stop()
            print("Target reached.")
            logger_a.info("Target reached.")
            break
            
        sleep(update_interval)
 
def is_target_reached(current_lat, current_lon, target_lat, target_lon, speed, threshold):
    #tolerance = 0.0000001
    current_pos = (current_lat, current_lon)
    target_pos = (target_lat, target_lon)
    t_reached = False
    cur_distance = haversine(target_pos, current_pos, unit=Unit.METERS) 
    #target_reached = cur_distance < threshold


    '''
    diff_lat = abs(target_pos[0] - current_pos[0])
    diff_lon = abs(target_pos[1] - current_pos[1])
    #print('{:f}'.format(diff_lat))
    #print('{:f}'.format(diff_lon))
    print("diff_lat:", diff_lat)
    print(diff_lon)
    print("threshold:", threshold)
    #print('{:f}'.format(threshold))
    if diff_lat <= threshold and diff_lon <= threshold:
        print("Met Condition")
        target_reached = True
    
    '''
    # if finish_1_lat < current_lat < finish_2_lat and finish_1_lon < current_lon < finish_2_lon 

    #print("Current Pos:" + str(current_pos) + " Target Pos:" + str(target_pos) + " Threshold:" + str(threshold))
    #logger_a.info("Current Pos:" + str(current_pos) + " Target Pos:" + str(target_pos) + " Threshold:" + str(threshold))
    #print("Current Distance:" + str(cur_distance))
    #logger_a.info("Current Distance:" + str(cur_distance))
    #print("Target Reached:" + str(target_reached))
    #logger_a.info("Target Reached:" + str(target_reached))

    #return target_reached
    t_reached = calculate_arrival_time(cur_distance, speed)
    return t_reached

'''
def is_target_reached(current_lat, current_lon, target_lat, target_lon, distance_traveled, threshold):
    current_pos = (current_lat, current_lon)
    target_pos = (target_lat, target_lon)
    starting_distance = haversine(target_pos, current_pos, unit=Unit.METERS)
    ending_target = starting_distance - distance_traveled
    target_reached = ending_target < threshold
    print("Target Reached:" + str(target_reached))
    logger_a.info("Target Reached:" + str(target_reached))
    return target_reached
'''

def calculate_arrival_time(distance, speed):
    target_reached = False
    
    time_elapsed = time() - start_time
    speed_m_s = speed / 2.237
    #target_time = (distance / speed_m_s) + start_time
    target_time = (distance / speed_m_s)
    #print("time_elapsed:", time_elapsed)
    print("Elapsed Time: {:.0f} seconds".format(time_elapsed))
    #print("target_time:", target_time)
    print("Target Time: {:.0f} seconds".format(target_time))
    if time_elapsed >= target_time:
        target_reached = True
    print("target_reached:", target_reached)
    return target_reached

def calculate_new_position(lat, lon, bearing, distance):
    """
    Calculate new position given a start point, bearing and distance.
    
    Args:
    lat (float): Latitude of the starting point.
    lon (float): Longitude of the starting point.
    bearing (float): Bearing in degrees.
    distance (float): Distance to move in meters.
    
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


# Will be used to get target coordinates from current lat, lon and distance needed
def new_coordinates(lat, lon, distance_meters, direction_degrees):
    # Constants
    meters_per_degree = 111139  # Approximate number of meters per degree of latitude
    
    # Convert direction degrees to radians
    direction_radians = math.radians(direction_degrees)
    
    # Calculate the change in latitude
    delta_lat = (distance_meters * math.cos(direction_radians)) / meters_per_degree
    
    # Calculate the change in longitude, considering the latitude change
    latitude_average = math.radians(lat + delta_lat / 2)  # Average latitude in radians
    meters_per_degree_longitude = meters_per_degree * math.cos(latitude_average)
    delta_lon = (distance_meters * math.sin(direction_radians)) / meters_per_degree_longitude
    
    # Update the latitude and longitude
    new_lat = lat + delta_lat
    new_lon = lon + delta_lon
    
    return new_lat, new_lon


def navigate(target_locations, challenge):
    initial_heading = get_updating_degree()
    print("Initial Heading:", initial_heading)
    logger_a.critical("Initial Heading:" + str(initial_heading))
    print("Challenge Route:", challenge)
    
    start_time = time()  # Assuming you want the current time from `time()`
    logger_a.critical("Start Time:", str(start_time))

    # Define courses for challenges
    challenge_courses = {
        2: [0],  # Straight path
        3: [0, 90, -90],  # S turn within the swim lane
        4: [0, -90, 90]  # Bigger S turn swapping lanes
    }
    
    turn_degrees = challenge_courses.get(challenge, [0])  # Default to [0] if challenge is not defined
    counter = 0
    #current_lat = 33.8490934
    #current_lon = -118.0661198
    
    for location, (latitude, longitude) in target_locations.items():
        print(f"Navigating to: {location}")
        ugv_controls = UGVControls()  # Assuming initialization of UGV control here
        move_ugv_to_target(current_lat, current_lon, latitude, longitude, ugv_controls, initial_heading)
        print(f"Target location {location} reached.")
        
        end_time = time()
        time_elapsed = end_time - start_time
        logger_a.critical("End Time:" + str(end_time))   
        logger_a.critical("Time Elapsed:" + str(time_elapsed))
        
        counter += 1
        if counter < len(turn_degrees):
            initial_heading += turn_degrees[counter]
            current_lat = latitude
            current_lon = longitude
        
        sleep(0.5)  # Ensure this is from `time` module