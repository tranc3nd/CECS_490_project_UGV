'''
  Module to retrieve wifi status.
'''

from subprocess import check_output
from threading import Thread
import threading
from queue import Queue
import paho.mqtt.client as mqtt


# Create Publish connection to websockets
topic = "Wifi_pub"
msg_count = 0
broker_hostname ="localhost"
port = 9001 
def on_connect(client, userdata, flags, return_code):
    if return_code == 0:
        #print("connected")
        None
    else:
        print("could not connect, return code:", return_code)
client = mqtt.Client("Clientwifistat", transport='websockets')
# client.username_pw_set(username="user_name", password="password") # uncomment if you use password auth
client.on_connect = on_connect
client.connect(broker_hostname, port)
client.loop_start()


def getStatus():
    #while True:
    mode = check_output("iwconfig wlan0 | grep 'Mode' | head -1", shell=True, text=True)
    ssid = check_output("iwconfig wlan0 | grep 'ESSID' | head -1", shell=True, text=True)
    bit = check_output("iwconfig wlan0 | grep 'Bit' | head -1", shell=True, text=True)
    link = check_output("iwconfig wlan0 | grep 'Link' | head -1", shell=True, text=True)
    wifiData = [ mode, ssid, bit, link ]
    print(wifiData)
    client.publish(topic, str(wifiData))

def start_reading_wifi():
    thread = Thread(target=getStatus, daemon=True)
    thread.start()
  
if __name__ == '__main__':
  while True: 
    start_reading_wifi()