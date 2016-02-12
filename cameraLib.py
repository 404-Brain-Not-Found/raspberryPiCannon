# import the necessary packages
from __future__ import print_function
import motionDetection
from imutils.video import VideoStream
import numpy as np
import datetime
import imutils
import time
import cv2
 
# initialize the video streams and allow them to warmup
print("[INFO] starting cameras...")
webcam = VideoStream(src=0).start()
picam = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
 
# initialize the two motion detectors, along with the total
# number of frames read
baseCam = BasicMotionDetector()
servoCam = BasicMotionDetector()
total = 0
