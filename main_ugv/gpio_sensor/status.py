'''
  Module to route all GPIO statuses.
'''

from flask import Flask, render_template, Blueprint, request, session
from gpio_sensor.object_detect.detect import detectstatus
from gpio_sensor.wifi_detect.wifi_status import wifimode, wifibit, wifilink, wifissid


status_page = Blueprint('status', __name__)
@status_page.route('/', methods=['GET'])
def status():
  # Object Status Details
  templateData = detectstatus()
  
  # Wifi Status Details
  templateData.update({'wifi_mode': wifimode()})
  templateData.update({'wifi_bit': wifibit()})
  templateData.update({'wifi_link': wifilink()})
  templateData.update({'wifi_ssid': wifissid()})

  return render_template('index.html', ** templateData)