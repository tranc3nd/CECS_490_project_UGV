#!/bin/bash

# Define the directory where your app is located
APP_DIR="/home/jason/Documents/github/master/main_ugv/com"
#APP_DIR="/app/main_ugv/com"

# Define the name of the application module
APP_MODULE="main:app"

# Number of worker processes
THREADS=10

# IP address and port
ADDRESS=0.0.0.0:8000

# Navigate to the application directory
cd $APP_DIR

# Activate the virtual environment if you have one
# source venv/bin/activate

# Start Gunicorn with the specified module and number of workers
gunicorn --threads=$THREADS $APP_MODULE --bind=$ADDRESS
