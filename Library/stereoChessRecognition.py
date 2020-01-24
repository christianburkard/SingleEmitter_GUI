"""
Created on 22.01.2020

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


def startChessCycle():
    # Photo session settings
    total_photos = 30             # Number of images to take
    countdown = 2                 # Interval for count-down timer, seconds
    font=cv2.FONT_HERSHEY_SIMPLEX # Cowntdown timer font

    # Camera settimgs
    cam_width = 1024              # Cam sensor width settings
    cam_height = 512              # Cam sensor height settings

    # Final image capture settings
    scale_ratio = 1.0

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
    if not args.get("video", False):
        print("Starting video stream...")
        vs1 = VideoStream(src=0).start()
        vs2 = VideoStream(src=1).start()
    #    vs = WebcamVideoStream(src=0).start()

    # Lets start taking photos!
    counter = 0
    framewidth = 1024
    t2 = datetime.now()
    print ("Starting photo sequence")
    #for frame in camera.capture_continuous(capture, format="bgra", \
    #                  use_video_port=True, resize=(img_width,img_height)):
    while True:
        frame1 = vs1.read()
        frame2 = vs2.read()
        # handle the frame from VideoCapture or VideoStream
        frame1 = frame1[1] if args.get("video", False) else frame1
        frame1 = imutils.resize(frame1, width=framewidth)
        cv2.imshow("pair1", frame1)
        frame2 = frame2[1] if args.get("video", False) else frame2
        frame2 = imutils.resize(frame2, width=framewidth)
        cv2.imshow("pair2", frame2)
        t1 = datetime.now()
        cntdwn_timer = countdown - int ((t1-t2).total_seconds())
        # If cowntdown is zero - let's record next image
        if cntdwn_timer == -1:
          counter += 1
          filename1 = './Images/Chessboard/scene1_'+str(img_width)+'x'+str(img_height)+'_'+\
                      str(counter) + '.png'
          filename2 = './Images/Chessboard/scene2_'+str(img_width)+'x'+str(img_height)+'_'+\
                      str(counter) + '.png'
          cv2.imwrite(filename1, frame1)
          cv2.imwrite(filename2, frame2)
          print (' ['+str(counter)+' of '+str(total_photos)+'] '+filename1)
          print (' ['+str(counter)+' of '+str(total_photos)+'] '+filename2)
          t2 = datetime.now()
          time.sleep(1)
          cntdwn_timer = 0      # To avoid "-1" timer display
          next
        # Draw cowntdown counter, seconds
        cv2.putText(frame1, str(cntdwn_timer), (50,50), font, 2.0, (0,0,255),4, cv2.LINE_AA)
        cv2.imshow("pair1", frame1)
        cv2.putText(frame2, str(cntdwn_timer), (50,50), font, 2.0, (0,0,255),4, cv2.LINE_AA)
        cv2.imshow("pair2", frame2)
        key = cv2.waitKey(1) & 0xFF

        # Press 'Q' key to quit, or wait till all photos are taken
        if (key == ord("r")) | (counter == total_photos):
          break


    print ("Photo sequence finished")
    cv2.destroyAllWindows()