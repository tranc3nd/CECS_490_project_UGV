'''
  Module to send controls to UGV.
'''

# Import required libraries
from flask import Flask, render_template, Blueprint, request,jsonify, abort
import requests
from gpio_sensor.motor.manual_mov import manual_dir, setSpeed, initialize_ultra_sonic_feed
from threading import Thread
from gpio_sensor.motor.motor_navi import live_gps_data, initialize_cur_loc


mode_page = Blueprint('modeone', __name__)

@mode_page.route('/mode', methods=['GET'])
def mode_oneindex():
  initialize_cur_loc()
  initialize_ultra_sonic_feed()
  return render_template('mode.html')

@mode_page.route('/mode/speed', methods=['POST'])
def mode_speed_control():
    try:
        postData = request.get_data()
        strData = postData.decode("utf-8")
        result = setSpeed(strData)
        return jsonify({"status": "success", "data": strData, "result": result}), 200
    except Exception as e:
        # Log the error and abort with a 500 internal server error
        print(f"Error processing request: {e}")
        abort(500, description="Internal Server Error")

@mode_page.route('/mode/control', methods=['POST'])
def mode_control():
    try:
        postData = request.get_data()
        strData = postData.decode("utf-8")
        result = manual_dir(strData)
        return jsonify({"status": "success", "data": strData, "result": result}), 200
    except Exception as e:
        # Log the error and abort with a 500 internal server error
        print(f"Error processing request: {e}")
        abort(500, description="Internal Server Error")

# Destination point navigation
@mode_page.route('/mode/destinations', methods=['POST'])
def mode_destinations():
    try:
        data = request.json
        challenge = int(data['challenge'])
        lat1 = float(data['lat1'])
        lng1 = float(data['long1'])
        lat2 = float(data['lat2'])
        lng2 = float(data['long2'])
        lat3 = float(data['lat3'])
        lng3 = float(data['long3'])
        target_array = { 
            "Destination1": (lat1, lng1), 
            "Destination2": (lat2, lng2), 
            "Destination3": (lat3, lng3)
        }

        result = live_gps_data(target_array, challenge)
        return jsonify({"status": "success", "data": target_array, "result": result}), 200
    except Exception as e:
        # Log the error and abort with a 500 internal server error
        print(f"Error processing request: {e}")
        abort(500, description="Internal Server Error")        

# Map click navigation
@mode_page.route('/mode/map', methods=['POST'])
def mode_map():
    data = request.json
    challenge = int(data['challenge'])
    lat = float(data['lat'])
    lng = float(data['long'])
    print('Latitude:', lat)
    print('Longitude:', lng)
    target_location = { 
        "Destination1": (lat, lng)
    }
    live_gps_data(target_location, challenge)
    
    if lat is not None and lng is not None:
           # Send only lat and lng to the Pathfinder program
           pathfinder_data = {'lat': lat, 'long': lng}
    try:
        return jsonify({'message': 'Data sent successfully to pathfinder program'})
    except requests.exceptions.RequestException as e:
        print('Error:', e)
        return jsonify({'error': 'Failed to send data to pathfinder program'}), 500
      