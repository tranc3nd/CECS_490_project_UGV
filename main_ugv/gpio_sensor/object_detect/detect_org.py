import argparse
import sys
import time
import queue

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from threading import Thread
import concurrent.futures
import threading
import paho.mqtt.client as mqtt
import json
import base64

from queue import Queue


'''
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
'''

from gpio_sensor.object_detect.utils import visualize
#from utils import visualize
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Initialize variables
x_location = ''
location = ''
parser = ''
score = ''
object = ''
templateData = ''


# custom thread
class CustomThread(Thread):
    # constructor
    def __init__(self):
        # execute the base constructor
        Thread.__init__(self)
        # set a default value
        self.value = None
 
    # function executed in a new thread
    def run(self):
        # block for a moment
        time.sleep(1)
        # store data in an instance variable
        self.value = Thread

def run(model: str, max_results: int, score_threshold: float, 
    camera_id: int, width: int, height: int, enable_edgetpu: bool, num_threads: int) -> None:
  


  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    max_results: Max number of detection results.
    score_threshold: The score threshold of detection results.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
  """



  # Visualization parameters
  row_size = 50  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 0)  # black
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  detection_frame = None
  detection_result_list = []
  # Initialize global variables
  global object
  global x_location
  global score
  global location
  global templateData

  def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
    global FPS, COUNTER, START_TIME


    # Calculate the FPS
    if COUNTER % fps_avg_frame_count == 0:
        FPS = fps_avg_frame_count / (time.time() - START_TIME)
        START_TIME = time.time()

    detection_result_list.append(result)
    COUNTER += 1


  # Initialize the object detection model
  #base_options = core.BaseOptions(file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  base_options = python.BaseOptions(model_asset_path=model)
  options = vision.ObjectDetectorOptions(base_options=base_options,
                                         running_mode=vision.RunningMode.LIVE_STREAM,
                                         max_results=max_results, score_threshold=score_threshold,
                                         result_callback=save_result)
  detector = vision.ObjectDetector.create_from_options(options)
  


  # Continuously capture images from the camera and run inference
  while True:
    im= picam2.capture_array()  
#    success, image = cap.read()
    image=cv2.resize(im,(640,480))
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

    # Run object detection using the model.
    detector.detect_async(mp_image, time.time_ns() // 1_000_000)

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(FPS)
    text_location = (left_margin, row_size)
    current_frame = image
    getCv2 = cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                font_size, text_color, font_thickness, cv2.LINE_AA)
    #success, encoded_image = cv2.imencode('.jpg', getCv2)
    #jpg_as_text = base64.b64encode(encoded_image)
    #client.publish("Stream_video", jpg_as_text)

    if detection_result_list:

        # Get detected object and score
        if len(detection_result_list[0].detections) == True:

                # Move UGV to left
            #print("Object: ", object)
            #print("Score: ", score)
            #print("x_location: ", x_location)
            if(detection_result_list[0].detections[0].categories[0].score > float(0.50)):
                object = detection_result_list[0].detections[0].categories[0].category_name
                score = format(detection_result_list[0].detections[0].categories[0].score, ".0%")
                x_location = detection_result_list[0].detections[0].bounding_box.origin_x

                # Determine Object location
                if x_location < 150:
                    location = 'Left'
                    # Move UGV to right
                elif x_location >= 150:
                    location = 'Right'
                templateData = [ object, score, location ]
                #q.put_nowait(templateData)
                client.publish(topic, str(templateData))
            #yield templateData
           

        current_frame = visualize(current_frame, detection_result_list[0])
        detection_frame = current_frame
        detection_result_list.clear()

    #if detection_frame is not None:
        #cv2.imshow('object_detection', detection_frame)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
        break

  detector.close()
  #cap.release()
  cv2.destroyAllWindows()

#def main():
def parameters():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      #default='../gpio_sensor/object_detect/efficientdet_lite0_edgetpu.tflite')
      default='../gpio_sensor/object_detect/efficientdet_lite0.tflite')
      #default='efficientdet_lite0.tflite')
      #default='best.tflite')
  parser.add_argument(
      '--maxResults',
      help='Max number of detection results.',
      required=False,
      default=5)
  parser.add_argument(
      '--scoreThreshold',
      help='The score threshold of detection results.',
      required=False,
      type=float,
      default=0.25)
  # Finding the camera ID can be very reliant on platform-dependent methods. 
  # One common approach is to use the fact that camera IDs are usually indexed sequentially by the OS, starting from 0. 
  # Here, we use OpenCV and create a VideoCapture object for each potential ID with 'cap = cv2.VideoCapture(i)'.
  # If 'cap' is None or not 'cap.isOpened()', it indicates the camera ID is not available.
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=True)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=2)
  return parser


q = Queue()
semaphore = threading.Semaphore(value=5)

def detectstatus():
    args, unknown = parameters().parse_known_args()
    run(args.model, int(args.maxResults), args.scoreThreshold, int(args.cameraId), args.frameWidth, args.frameHeight, bool(args.enableEdgeTPU), int(args.numThreads))

def start_reading_object():
    thread = Thread(target=detectstatus, daemon=True)
    thread.start()
     

# Create Publish connection to websockets
topic = "Object_pub"
msg_count = 0
broker_hostname ="localhost"
port = 9001 
def on_connect(client, userdata, flags, return_code):
    if return_code == 0:
        print("connected")
    else:
        print("could not connect, return code:", return_code)
client = mqtt.Client("Client1", transport='websockets')
# client.username_pw_set(username="user_name", password="password") # uncomment if you use password auth
client.on_connect = on_connect
client.connect(broker_hostname, port)
client.loop_start()


    
if __name__ == '__main__':
  while True:
    start_reading_object()