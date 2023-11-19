'''
	Raspberry Pi Main Driver Class
  To run web server: gunicorn main:app --bind=0.0.0.0:8000 --threads 4
'''

# Import Flask and sys Libraries
from flask import Flask
from pathlib import Path
from sys import path


# Import GPIO paths
object_dir = str(Path.cwd().parent) + '/gpio_sensor/object_detect'
path.insert(0, object_dir)
from gpio_sensor.status import status_page
from mode.mode import mode_page

# Set flask paths for template and static directory
template_dir = str(Path.cwd().parent) + '/com/templates'
app = Flask(__name__, template_folder=template_dir)
app.static_folder = str(Path.cwd().parent) + '/com/static'


# Challenge Modes
app.register_blueprint(mode_page)

# Status of all GPIO Sensor readings
app.register_blueprint(status_page)


if __name__ == "__main__":
  app.run(host='0.0.0.0', port=8000)