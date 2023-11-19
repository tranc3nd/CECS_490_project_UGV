'''
  Module to retrieve wifi status.
'''
from flask import Flask, redirect, url_for, session
from subprocess import check_output, run

wifiresult = ''
status = ''
strength = ''

def wifimode():
  mode = check_output("iwconfig wlan0 | grep 'Mode' | head -1", shell=True, text=True)
  return mode

def wifissid():
  ssid = check_output("iwconfig wlan0 | grep 'ESSID' | head -1", shell=True, text=True)
  return ssid

def wifibit():
  bit = check_output("iwconfig wlan0 | grep 'Bit' | head -1", shell=True, text=True)
  return bit

def wifilink():
  link = check_output("iwconfig wlan0 | grep 'Link' | head -1", shell=True, text=True)
  return link
