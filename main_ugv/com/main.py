'''
	Raspberry Pi Main Driver Class
'''
# Import Flask and sys Libraries
from flask import Flask, render_template, redirect
from pathlib import Path
from sys import path
import gunicorn

# Insert object_detect path and import thread
object_dir = str(Path.cwd().parent) + '/gpio_sensor/object_detect'
path.insert(0, object_dir)
from detect import createObjDetThread

# Set flask paths for template and static directory
template_dir = str(Path.cwd().parent) + '/com/templates'
app = Flask(__name__, template_folder=template_dir)
app.static_folder = str(Path.cwd().parent) + '/com/static'


# Main class that will run multi threaded services for GPIO
class main:
  app = Flask(__name__)

  # Status of all GPIO Sensor readings 
  @app.route("/", methods=['GET']) 
  def statusindex():  
    thread_one = createObjDetThread()
    thread_one.start()
    thread_one.join()
    templateData = thread_one.value
    print('test: ',templateData)
    return render_template('index.html', ** templateData)
  
  # Challenge Modes
  @app.route("/mode/", methods=['GET', 'POST']) 
  def mode():
    thread_two = None
    #thread_two.start()
    #thread_two.join()
    mode = None
    return render_template('mode.html') 
 
 
  if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8000, debug=False)
    
   
