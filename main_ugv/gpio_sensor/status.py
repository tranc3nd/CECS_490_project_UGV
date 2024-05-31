import time
import concurrent.futures
from queue import Empty
import paho.mqtt.client as mqtt
import threading

from flask import Flask, jsonify, render_template, Blueprint, request, session, abort, jsonify
#from gpio_sensor.object_detect.detect import start_reading_object
#from gpio_sensor.wifi_detect.wifi_status import start_reading_wifi
#from gpio_sensor.esp32.read_esp32_data import run_esp
from gpio_sensor.compass.heading_detect import run_compass, stop_compass
from gpio_sensor.motor.speed_control_3 import setup
from gpio_sensor.ultrasonic_detect.ultrasonic_status import start_reading_dist


# Function to stop the threads
def stop_threads():
    stop_event.set()

# Initialize the stop event
stop_event = threading.Event()


# Initialize the data queues
distance_detect = start_reading_dist()
#wifi_detect = start_reading_wifi()
#esp32_detect = run_esp()
compass_detect = run_compass()
setup_LR_motor = setup()
'''
def read_esp32():
    try:
        return esp32_detect.get(timeout=1)
    except Empty:
        return None
'''

def read_compass():
    try:
        return compass_detect.get(timeout=0.1)
    except KeyboardInterrupt:
        return stop_compass()

'''
def read_object():
    try:
        return object_detect.get(timeout=0.01)  # Adjust timeout as needed
    except Empty:
        return None
'''
'''
def read_wifi():
    try:
        return wifi_detect.get(timeout=1)
    except Empty:
        return None
'''

def read_distance():
    try:
        return distance_detect.get(timeout=1)  # Adjust timeout as needed
    except Empty:
        return None


status_page = Blueprint('status', __name__)

@status_page.route('/', methods=['GET'])
def status():
    #start_reading_object()
    #read_compass()

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Submit tasks to the executor for concurrent execution
        #executor.submit(read_wifi)
        #executor.submit(read_esp32)
        executor.submit(read_compass)
        executor.submit(setup_LR_motor)
        executor.submit(read_distance)
        
    # Prepare the template data with the fetched statuses
    templateData = {}
    
    return render_template('index.html', **templateData)


@status_page.route('/reset', methods=['POST'])
def resetUGV():
    try:
        
        print("Stopping threads...")
        stop_threads()

        time.sleep(5)

        print("Starting threads...")

        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Submit tasks to the executor for concurrent execution
            #executor.submit(read_wifi)
            #executor.submit(read_esp32)
            executor.submit(read_compass)
            executor.submit(setup_LR_motor)
            executor.submit(read_distance)
            postData = request.get_data()
            strData = postData.decode("utf-8")
            return jsonify({"status": "success", "data": strData}), 200
    except Exception as e:
        # Log the error and abort with a 500 internal server error
        print(f"Error processing request: {e}")
        abort(500, description="Internal Server Error")
        