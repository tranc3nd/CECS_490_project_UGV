'''
  Module to route all GPIO statuses.
'''

from flask import Flask, render_template, Blueprint, request, session
#from gpio_sensor.object_detect.detect import detectstatus
from gpio_sensor.moist_detect.moist_detect import getHitStatus
from gpio_sensor.wifi_detect.wifi_status import wifimode, wifibit, wifilink, wifissid


status_page = Blueprint('status', __name__)
@status_page.route('/', methods=['GET'])
def status():
  # Object Status Details
  #templateData = detectstatus()
  templateData = {}
  
  # Wifi Status Details
  templateData.update({'wifi_mode': wifimode()})
  templateData.update({'wifi_bit': wifibit()})
  templateData.update({'wifi_link': wifilink()})
  templateData.update({'wifi_ssid': wifissid()})

  #Moist Status Details 
  templateData.update({'moist_detect': getHitStatus()})
  return render_template('index.html', ** templateData)