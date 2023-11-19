'''
  Module runs the object detect routine.
'''

import argparse
from collections.abc import Callable, Iterable, Mapping
import sys
from pathlib import Path
import time
from typing import Any

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

from time import sleep
from threading import Thread



# Insert templates path and import thread
templates_dir = str(Path.cwd().parent) + '/com/templates/'


# Initialize variables
location = ''
parser = ''
score = ''
object = ''
templateData = ''


def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 30


  # Initialize global variables
  global object
  global x_location
  global score
  global location

  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)


  # Continuously capture images from the camera
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    counter += 1
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)

    # Draw keypoints and edges on input image
    image = utils.visualize(image, detection_result)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
      break
    #cv2.imshow('object_detector', image)
    
    # Determine detection results with camera
    if len(detection_result.detections) == True:
      object = detection_result.detections[0].categories[0].category_name
      score = format(detection_result.detections[0].categories[0].score, ".0%")
      x_location = detection_result.detections[0].bounding_box.origin_x
      
      if detection_result.detections[0].categories[0].score > float(0.70):

        # Debugging only
        #print("\n")
        #print("Object:", object)
        #print("Score Guess:", score)
        #print(fps_text)
      
        # Determine Object location
        if x_location < 150:
          location = 'Left'
          # Move UGV to right
        elif x_location >= 150:
          location = 'Right'
          # Move UGV to left

  cap.release()
  cv2.destroyAllWindows()


def parameters():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='../gpio_sensor/object_detect/efficientdet_lite0_edgetpu.tflite')
      #default='efficientdet_lite0_edgetpu.tflite')
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
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=True)
  
  return parser


def detectstatus():
  args, unknown = parameters().parse_known_args()
  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight, int(args.numThreads), bool(args.enableEdgeTPU) )
  templateData = {
    'object': object,
    'score': score,
    'location': location
  }
  return templateData

  
