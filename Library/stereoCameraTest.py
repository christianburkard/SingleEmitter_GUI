"""
Created on 20.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""

from __future__ import print_function
from imutils.video import VideoStream
from imutils.video import FPS
#from Library.ThreadsLib import FPSOutput
#from Library.ThreadsLib import WebcamVideoStream
from imutils.video.pivideostream import PiVideoStream
import tkinter as tk
from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2
import imutils
from collections import deque
import time
import dlib
import csv
import datetime
import time
import threading
import queue
import sys
import random
import serial
import pandas as pd
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import tkinter as tk
from tkinter import filedialog, Canvas
import keyboard
from tkinter.ttk import Frame, Button, Entry, Style
from concurrent import futures
import time
import cv2
import numpy as np
import os
from datetime import datetime

def stereoCameraTest():
    # File for captured image
    filename = './scenes/photo.png'

    # Camera settimgs
    cam_width = 1024
    cam_height = 512

    # Final image capture settings
    scale_ratio = 0.5

    # Camera resolution height must be dividable by 16, and width by 32
    cam_width = int((cam_width+31)/32)*32
    cam_height = int((cam_height+15)/16)*16
    print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

    # Buffer for captured image settings
    img_width = int (cam_width * scale_ratio)
    img_height = int (cam_height * scale_ratio)
    capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
    print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

    # Initialize the camera
    ap = argparse.ArgumentParser()
    args = vars(ap.parse_args())


    t2 = datetime.now()
    counter = 0
    avgtime = 0
    framewidth = 1024

    if not args.get("video", False):
        print("Starting video stream...")
#        vs1 = VideoStream(src=0).start()
#        vs2 = VideoStream(src=1).start()
        cap1 = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
    #    vs = WebcamVideoStream(src=0).start()

    # Capture frames from the camera
    while True:
        t1 = datetime.now()
        timediff = t1-t2
        avgtime = avgtime + (timediff.total_seconds())
#        frame1 = vs1.read()
#        frame2 = vs2.read()
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        cap1.set(3,400)
        cap2.set(4,400)
        # handle the frame from VideoCapture or VideoStream
        frame1 = frame1[1] if args.get("video", False) else frame1
        frame1 = imutils.resize(frame1, width=framewidth)

        cv2.imshow("pair1", frame1)
        frame2 = frame2[1] if args.get("video", False) else frame2
        frame2 = imutils.resize(frame2, width=framewidth)
        cv2.imshow("pair2", frame2)
        key = cv2.waitKey(1) & 0xFF
        t2 = datetime.now()
        counter = counter + 1
        # if the `q` key was pressed, break from the loop and save last image
        if key == ord("r") :
            avgtime = avgtime/counter
            print ("Average time between frames: " + str(avgtime))
            print ("Average FPS: " + str(1/avgtime))
    #        if (os.path.isdir("./scenes")==False):
    #            os.makedirs("./scenes")
    #        cv2.imwrite(filename, frame)
            break

    cv2.destroyAllWindows()

