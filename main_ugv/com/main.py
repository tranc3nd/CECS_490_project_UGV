'''
	Raspberry Pi Main Driver Class
  To run web server: gunicorn main:app --bind=0.0.0.0:8000 --threads=9
'''

# Import Flask and sys Libraries
from flask import Flask
from pathlib import Path
from sys import path
import json
from flask_mqtt import Mqtt
from flask_socketio import SocketIO





# Import GPIO paths
object_dir = str(Path.cwd().parent)
path.insert(0, object_dir)
from gpio_sensor.status import status_page
from mode.mode import mode_page

# Set flask paths for template and static directory
template_dir = str(Path.cwd().parent) + '/com/templates'
app = Flask(__name__, template_folder=template_dir)
app.static_folder = str(Path.cwd().parent) + '/com/static'
app.config.from_pyfile('config.py')

app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['MQTT_BROKER_URL'] = 'localhost'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''
app.config['MQTT_PASSWORD'] = ''
app.config['MQTT_KEEPALIVE'] = 5
app.config['MQTT_TLS_ENABLED'] = False


# Challenge Modes
app.register_blueprint(mode_page)

# Status of all GPIO Sensor readings
app.register_blueprint(status_page)


mqtt = Mqtt(app)
socketio = SocketIO(app)

@socketio.on('publish')
def handle_publish(json_str):
    data = json.loads(json_str)
    mqtt.publish(data['topic'], data['message'])

@socketio.on('subscribe', namespace='/status')
def handle_subscribe(json_str):
    data = json.loads(json_str)
    print(data)
    mqtt.subscribe(data['topic'])

@socketio.on('unsubscribe_all')
def handle_unsubscribe_all():
    mqtt.unsubscribe_all()

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    socketio.emit('mqtt_message', data=data)


if __name__ == "__main__":
  #app.run(host='0.0.0.0', port=8000)
  socketio.run(app, host='0.0.0.0', port=8000, use_reloader=True)