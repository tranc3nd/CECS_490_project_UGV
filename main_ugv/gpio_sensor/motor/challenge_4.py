'''
from gpiozero import Motor, PWMOutputDevice, Button
from time import sleep
from pathlib import Path
from sys import path
import serial
from loguru import logger
from gpiozero import Buzzer
from gpiozero import LED
import concurrent.futures


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

# Motor and PWM setup
motor_Ra = Motor(forward=20, backward=19)
motor_La = Motor(forward=12, backward=13)
pwm_a = PWMOutputDevice(pin=21, frequency=100)
pwm_b = PWMOutputDevice(pin=6, frequency=100)

#Encoder setup using gpiozero Button to detect rising edges
encoder_Ra = Button(22)
#encoder_Rb = Button(23)

encoder_La = Button(24)
#encoder_Lb = Button(25)

#Encoder counters
counter_Ra = 0
counter_La = 0

# Constants for calculation
pulses_per_revolution = 616  # Adjust this to your encoder's specification
wheel_circumference = 15.7 # Meter (adjust to your wheel's circumference)

# Increment counter on encoder pulse
def increment_counter_a():
    global counter_Ra
    counter_Ra += 1

def increment_counter_b():
    global counter_La
    counter_La += 1

def stop():
    motor_Ra.stop()
    motor_La.stop()
    pwm_a.value = 0
    pwm_b.value = 0

def pivot_right():
    reset_counters()
    motor_Ra.backward()
    motor_La.forward()
    pwm_a.value = 0.5
    pwm_b.value = 0.45
    sleep(2)  # Adjust the duration as needed
    stop()

def pivot_left():
    reset_counters()
    motor_Ra.forward()
    motor_La.backward()
    pwm_a.value = 0.5
    pwm_b.value = 0.45
    sleep(2)  # Adjust the duration as needed
    stop()

# Attach the increment function to the encoder's "when_pressed" event
encoder_Ra.when_pressed = increment_counter_a
encoder_La.when_pressed = increment_counter_b

def calculate_distance(counter, pulses_per_revolution, wheel_circumference):
    revolutions = counter / pulses_per_revolution
    distance = revolutions * wheel_circumference  # Distance in centimeters
    return distance

def reset_counters():
    global counter_a, counter_b
    counter_Ra = 0
    counter_La = 0

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
            sleep(1)
            buzzer.off()
            read_out = "Water detected!"
            logger_a.critical("Water detected. UGV has been hit. Stopping UGV...")
            motor_Ra.stop()
            motor_La.stop()
            pwm_a.value = 0
            pwm_b.value = 0
            # Turn on the LED

        #print("LED is ON")
        #time.sleep(2)  # Delay to simulate processing; adjust as needed
    else:
        read_out = "No water detected."
    readings = [ read_out, hit_count, moisture ]

def read_challenge_4():
    while True:
        distance_goal = 30
        # Example usage   
        reset_counters()
        motor_Ra.forward()
        motor_La.forward()
        pwm_a.value = 0.7
        pwm_b.value = 0.71
        #while True:
        distance_a = calculate_distance(counter_Ra, pulses_per_revolution, wheel_circumference)
        distance_b = calculate_distance(counter_La, pulses_per_revolution, wheel_circumference)
        print(f"Distance traveled: Motor A: {distance_a}cm, Motor B: {distance_b}cm")
        sleep(0.4)
        if distance_a >= distance_goal or distance_b >= distance_goal:
            print(f"Target distance reached. Motor A: {distance_a}cm, Motor B: {distance_b}cm")
            motor_Ra.stop()
            motor_La.stop()
            pwm_a.value = 0
            pwm_b.value = 0
            break
        pivot_right()
        distance_goal = 50
        # Example usage   
        reset_counters()
        motor_Ra.forward()
        motor_La.forward()
        pwm_a.value = 0.7
        pwm_b.value = 0.71
        #while True:
        distance_a = calculate_distance(counter_Ra, pulses_per_revolution, wheel_circumference)
        distance_b = calculate_distance(counter_La, pulses_per_revolution, wheel_circumference)
        print(f"Distance traveled: Motor A: {distance_a}cm, Motor B: {distance_b}cm")
        sleep(0.4)
        if distance_a >= distance_goal or distance_b >= distance_goal:
            print(f"Target distance reached. Motor A: {distance_a}cm, Motor B: {distance_b}cm")
            motor_Ra.stop()
            motor_La.stop()
            pwm_a.value = 0
            pwm_b.value = 0
            break
        pivot_left()
        distance_goal = 30
        # Example usage   
        reset_counters()
        motor_Ra.forward()
        motor_La.forward()
        pwm_a.value = 0.7
        pwm_b.value = 0.71
        #while True:
        distance_a = calculate_distance(counter_Ra, pulses_per_revolution, wheel_circumference)
        distance_b = calculate_distance(counter_La, pulses_per_revolution, wheel_circumference)
        print(f"Distance traveled: Motor A: {distance_a}cm, Motor B: {distance_b}cm")
        sleep(0.4)
        if distance_a >= distance_goal or distance_b >= distance_goal:
            print(f"Target distance reached. Motor A: {distance_a}cm, Motor B: {distance_b}cm")
            motor_Ra.stop()
            motor_La.stop()
            pwm_a.value = 0
            pwm_b.value = 0
            ser.close()
            break
        

def challenge_4():
    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Submit tasks to the executor for concurrent execution
        executor.submit(read_esp_data)
        executor.submit(read_challenge_4)


if __name__ == '__main__':
    challenge_4()

    '''