
from gpiozero import Motor, PWMOutputDevice, Button
import time
from pathlib import Path
from sys import path
import serial
from loguru import logger
from gpiozero import Buzzer
from gpiozero import LED
import concurrent.futures
from threading import Event


buzzer = Buzzer(2) #GPIO 2
# Define the GPIO pin number
led_pin = 3

# Initialize the LED object with the GPIO pin number
led = LED(led_pin)

# Create Logger
logger_a = logger.bind(task="Moisture")
logger_a.add("../logs/moisture.log", rotation="500 MB", filter=lambda record: record["extra"]["task"] == "Moisture")

global hit_count

# Open serial connection to ESP32
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

object_dir = str(Path.cwd().parent)
path.insert(0, object_dir)

# Motor and PWM setup
motor_Ra = Motor(forward=20, backward=19)
motor_La = Motor(forward=13, backward=12)
pwm_a = PWMOutputDevice(pin=21, frequency=100)
pwm_b = PWMOutputDevice(pin=6, frequency=100)

#Encoder setup using gpiozero Button to detect rising edges
encoder_Ra = Button(25)
encoder_Rb = Button(24)
encoder_La = Button(22)
encoder_Lb = Button(23)

#Encoder counters
counter_Ra = 0
counter_La = 0

# Constants for calculation
pulses_per_revolution = 616  # Adjust this to your encoder's specification
wheel_circumference = 15.7 # Meter (adjust to your wheel's circumference)

Renca_t = 0
Rencb_t = 0
dtR = 0
Lenca_t = 0
Lencb_t = 0
dtL = 0

# Initialize variables to store data
latitude = None
longitude = None
siv = None
accuracy = None
moisture = None
event = Event()


def read_esp_data():
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
                print("Latitude:", latitude)
                print("Longitude:", longitude)
                print("SIV:", siv)
                print("Accuracy (m):", accuracy)
                print("Moisture:", moisture)
                read_gps(latitude, longitude, siv, accuracy)
                read_moist(moisture)

            

def read_gps(latitude, longitude, siv, accuracy):
    readings = [ latitude, longitude, siv, accuracy]
    #client_web.publish(gps_topic, str(readings))
    #client.publish(gps_topic, str(readings))

def read_moist(moisture):
    hit_count = 0
    if moisture < 2000:
        hit_count += 1
        buzzer.on()
        if hit_count == 1:
            #stop()
            led.on()
            time.sleep(1)
            buzzer.off()
            read_out = "Water detected!"
            logger_a.critical("Water detected. UGV has been hit. Stopping UGV...")
            #motor_Ra.stop()
            #motor_La.stop()
            #pwm_a.value = 0
            #pwm_b.value = 0
            # Turn on the LED

        #print("LED is ON")
        #time.sleep(2)  # Delay to simulate processing; adjust as needed
    else:
        read_out = "No water detected."
    readings = [ read_out, hit_count, moisture ]


# Increment counter on encoder pulse
def increment_counter_a():
    global Renca_t
    global counter_Ra
    Renca_t = time.time()
    counter_Ra += 1

def timer_Rencb():
    global Renca_t
    global Rencb_t
    global dtR
    Rencb_t = time.time()
    dtR = Renca_t - Rencb_t


def increment_counter_b():
    global Lenca_t
    global counter_La
    Lenca_t = time.time()
    counter_La += 1

def timer_Lencb():
    global Lenca_t
    global Lencb_t
    global dtL
    Lencb_t = time.time()
    dtL = Lenca_t - Lencb_t


# Attach the increment function to the encoder's "when_pressed" event
encoder_Ra.when_pressed = increment_counter_a
encoder_Rb.when_pressed = timer_Rencb
encoder_La.when_pressed = increment_counter_b
encoder_Lb.when_pressed = timer_Lencb

def calculate_distance(counter, pulses_per_revolution, wheel_circumference):
    revolutions = counter / pulses_per_revolution
    distance = revolutions * wheel_circumference  # Distance in centimeters
    return distance

def reset_counters():
    global counter_Ra, counter_La
    counter_Ra = 0
    counter_La = 0

def read_challenge_2():
    while True:
        distance_goal = 100
        # Example usage   
        reset_counters()
        motor_Ra.forward()
        motor_La.forward()
        pwm_a.value = 0.23
        pwm_b.value = 0.234
        #while True:
        distance_a = calculate_distance(counter_Ra, pulses_per_revolution, wheel_circumference)
        distance_b = calculate_distance(counter_La, pulses_per_revolution, wheel_circumference)
        time.sleep(0.4)
        while distance_a <= distance_goal or distance_b <= distance_goal:
            time.sleep(0.5)
            print(f"Motor A: {distance_a}cm, Motor B: {distance_b}cm, Speed: {dtR*39.23566}/sec")
            distance_a = calculate_distance(counter_Ra, pulses_per_revolution, wheel_circumference)
            distance_b = calculate_distance(counter_La, pulses_per_revolution, wheel_circumference)
        motor_Ra.stop()
        motor_La.stop()
        pwm_a.value = 0
        pwm_b.value = 0
        print(f"Distance traveled: Motor A: {distance_a}cm, Motor B: {distance_b}cm")
        break

    
def challenge_2():
    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Submit tasks to the executor for concurrent execution
        executor.submit(read_esp_data)
        executor.submit(read_challenge_2)


if __name__ == '__main__':
    challenge_2()



