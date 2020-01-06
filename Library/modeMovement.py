"""
Created on 03.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System
Class for different object movements
"""
# import the necessary packages
from __future__ import print_function
from imutils.video import VideoStream
from imutils.video import FPS
from Library.ThreadsLib import FPSOutput
from Library.ThreadsLib import WebcamVideoStream
from imutils.video.pivideostream import PiVideoStream
from Library.Calculation import Calculation
from Library.Functions import *
from Library.ArduinoPort import *
from Library.FFTPlot import *
from Library.pathFinder import *
from Library.settings import *
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
import tkinter as tk
from tkinter import filedialog, Canvas
import keyboard
from tkinter.ttk import Frame, Button, Entry, Style
from concurrent import futures


class modeMovement():

    def modeMovementUp(self):

        time.sleep(0.2)
        try:
             dataByte1, dataByte2, dataByte3 = defaultLookUp()
        except:
            #read in values from look-up table
            dataByte1, dataByte2, dataByte3 = defLookUpTable()

        stepIter = 0

#                time.sleep(0.1)

#                global ser
        ser = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Serial connection with FPGA established ...")
        time.sleep(0.2)
        stepNum = 720
        steps = 719
#                byte1 = 0

        try:
            frameWidth = int(globFrameWidth)
#                print("Frame width manually changed to: ", frameWidth)
        except:
            frameWidth = 720
            print("Default frame width : ", frameWidth)

#            deltaX = 400 #x-offset for centering image coordinate system
#            deltaY = 300 #y-offset for centering image coordinate system

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        args = vars(ap.parse_args())
        buffer = 120
        # define the lower and upper boundaries of the "black"

        try:
            blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
            print("Costumized HSV values entered ...")
        except:
            blackUpper = (100, 90, 90)
            print("Set HSV default values ")
        blackLower = (0, 0, 0)
        pts = deque(maxlen=buffer)

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not args.get("video", False):
            print("Starting video stream...")
#            vs = VideoStream(src=0).start()
            #-threaded videostream
            vs = WebcamVideoStream(src=0).start()
        # otherwise, grab a reference to the video file
        else:
            print(" No video stream possible")
        # allow the camera or video file to warm up
        time.sleep(0.5)
        tracker = None
        writer = None
        confdef = 0.2
        fps = FPSOutput().start()

        coordArrayX = np.array([])
        coordArrayY = np.array([])
        radiusArray = np.array([])
        timeArray = np.array([])
            # keep looping
#                while True:
        while stepIter >= 719:

            frame = vs.read()

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                print("No video preview possible")
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=frameWidth)
            height, width, channels = frame.shape
            deltaX = width/2 #x-offset for centering image coordinate system
            deltaY = height/2 #y-offset for centering image coordinate system
#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#            grayscale = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#
            # construct a mask for the color "black", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
        #    mask = cv2.inRange(hsv, greenLower, greenUpper)
        #    mask = cv2.erode(mask, None, iterations=2)
        #    mask = cv2.dilate(mask, None, iterations=2)

            # mask for black color
            mask1 = cv2.inRange(blurred, blackLower, blackUpper)
            mask2 = cv2.erode(mask1, None, iterations=2)

            #Apply median filter
            mask3 = cv2.dilate(mask2, None, iterations=1)
            mask4 = cv2.medianBlur(mask3,5)

            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

#            stopTimer()
            print("Contour length: ",len(cnts))
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if int(M["m00"]) == 0:
                    center = np.nan
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    radius = np.nan
                else:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    PixCoordX = (center[0]-deltaX)
                    PixCoordY = (center[1]-deltaY)*(-1)
                    radius = radius
                print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))

                # only proceed if the radius meets a minimum size
                if radius > 0.01:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

#                    elif radius < 20:

            # update the points queue
                try:
                    print("Contour radius: {:.2f}".format(radius))
                    PixRadius = radius
                except:
                    print("No radius detected")
                    PixCoordX = np.nan
                    PixCoordX = np.nan
            else:
                PixCoordX = np.nan
                PixCoordY = np.nan
                radius = np.nan
                PixRadius = radius
                print("PiX coordinate !!!!!!: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
                print("Contour radius: {:.2f}".format(radius))

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

            coordArrayX = np.append(coordArrayX,(PixCoordX))
            coordArrayY = np.append(coordArrayY,(PixCoordY))
            radiusArray = np.append(radiusArray,(PixRadius))
            timeArray = np.append(timeArray,time.time()) # time in seconds

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF


#                if self.modeChoice.get() == 0:
#                    funcMode1(serialObject)
#
#                if self.modeChoice.get() == 1:
#                    funcMode2(serialObject)
#
#                if self.modeChoice.get() == 2:
#                    funcMode3(serialObject,dataByte1,dataByte2,dataByte3)

            #set serial default 0 position values to fpga
            byte1 = dataByte1[stepIter]
            byte2 = dataByte2[stepIter]
            byte3 = dataByte3[stepIter]
            values = bytearray([byte1, byte2, byte3])
            print("Byte1: ",byte1)
            print("Byte2: ",byte2)
            print("Byte3: ",byte3)
#                ser = serialObject
            ser.write(values)
            print("Data bits written ...")
            stepIter = stepIter + 1
            print("Iteration step: ",stepIter)
#                    time.sleep(0.1)
            #print coordinate values on gui
#                printCoordsGui(self,PixCoordX,PixCoordY)

            # if the 'q' key is pressed, stop the loop
            if key == ord("r"):

                break

            # Update fps counter
            fps.update()

        # stop timer and disp. fps information
        fps.stop()
        print("Elapsed time: {:.2f}".format(fps.elapsed()))
        print("Approx. FPS: {:.2f}".format(fps.fps()))

        #writing pixel positions, time and radius in an array
        writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray)

        # if we are not using a video file, stop the camera video stream
        if not args.get("video", False):
            vs.stop()
        # otherwise, release the camera
        else:
            vs.release()
        # close all windows
        ser.close()
        cv2.destroyAllWindows()


    def modeMovementDown(self):

        time.sleep(0.2)


        try:
             dataByte1, dataByte2, dataByte3 = defaultLookUp()
        except:
            #read in values from look-up table
            dataByte1, dataByte2, dataByte3 = defLookUpTable()

#            byte1 = dataByte1[3]
#            byte2 = dataByte2[3]
#            byte3 = dataByte3[3]
        stepIter = 0

#                time.sleep(0.1)

#                global ser
        ser = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Serial connection with FPGA established ...")
        time.sleep(0.2)
        stepNum = 720
        steps = 719
#                byte1 = 0

        try:
            frameWidth = int(globFrameWidth)
#                print("Frame width manually changed to: ", frameWidth)
        except:
            frameWidth = 720
            print("Default frame width : ", frameWidth)

#            deltaX = 400 #x-offset for centering image coordinate system
#            deltaY = 300 #y-offset for centering image coordinate system

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        args = vars(ap.parse_args())
        buffer = 120
        # define the lower and upper boundaries of the "black"

        try:
            blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
            print("Costumized HSV values entered ...")
        except:
            blackUpper = (100, 90, 90)
            print("Set HSV default values ")
        blackLower = (0, 0, 0)
        pts = deque(maxlen=buffer)

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not args.get("video", False):
            print("Starting video stream...")
#            vs = VideoStream(src=0).start()
            #-threaded videostream
            vs = WebcamVideoStream(src=0).start()
        # otherwise, grab a reference to the video file
        else:
            print(" No video stream possible")
        # allow the camera or video file to warm up
        time.sleep(0.5)
        tracker = None
        writer = None
        confdef = 0.2
        fps = FPSOutput().start()

        coordArrayX = np.array([])
        coordArrayY = np.array([])
        radiusArray = np.array([])
        timeArray = np.array([])
            # keep looping
#                while True:

        for stepIter in  reversed(range(0,stepNum-1)):
            frame = vs.read()

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                print("No video preview possible")
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=frameWidth)
            height, width, channels = frame.shape
            deltaX = width/2 #x-offset for centering image coordinate system
            deltaY = height/2 #y-offset for centering image coordinate system
#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#            grayscale = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#
            # construct a mask for the color "black", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
        #    mask = cv2.inRange(hsv, greenLower, greenUpper)
        #    mask = cv2.erode(mask, None, iterations=2)
        #    mask = cv2.dilate(mask, None, iterations=2)

            # mask for black color
            mask1 = cv2.inRange(blurred, blackLower, blackUpper)
            mask2 = cv2.erode(mask1, None, iterations=2)

            #Apply median filter
            mask3 = cv2.dilate(mask2, None, iterations=1)
            mask4 = cv2.medianBlur(mask3,5)

            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

#            stopTimer()
            print("Contour length: ",len(cnts))
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if int(M["m00"]) == 0:
                    center = np.nan
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    radius = np.nan
                else:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    PixCoordX = (center[0]-deltaX)
                    PixCoordY = (center[1]-deltaY)*(-1)
                    radius = radius
                print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))

                # only proceed if the radius meets a minimum size
                if radius > 0.01:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

#                    elif radius < 20:

            # update the points queue
                try:
                    print("Contour radius: {:.2f}".format(radius))
                    PixRadius = radius
                except:
                    print("No radius detected")
                    PixCoordX = np.nan
                    PixCoordX = np.nan
            else:
                PixCoordX = np.nan
                PixCoordY = np.nan
                radius = np.nan
                PixRadius = radius
                print("PiX coordinate !!!!!!: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
                print("Contour radius: {:.2f}".format(radius))

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

            coordArrayX = np.append(coordArrayX,(PixCoordX))
            coordArrayY = np.append(coordArrayY,(PixCoordY))
            radiusArray = np.append(radiusArray,(PixRadius))
            timeArray = np.append(timeArray,time.time()) # time in seconds

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF


#                if self.modeChoice.get() == 0:
#                    funcMode1(serialObject)
#
#                if self.modeChoice.get() == 1:
#                    funcMode2(serialObject)
#
#                if self.modeChoice.get() == 2:
#                    funcMode3(serialObject,dataByte1,dataByte2,dataByte3)

            #set serial default 0 position values to fpga
            byte1 = dataByte1[stepIter]
            byte2 = dataByte2[stepIter]
            byte3 = dataByte3[stepIter]
            values = bytearray([byte1, byte2, byte3])
            print("Byte1: ",byte1)
            print("Byte2: ",byte2)
            print("Byte3: ",byte3)
#                ser = serialObject
            ser.write(values)
            print("Data bits written ...")
            stepIter = stepIter + 1
            print("Iteration step: ",stepIter)
#                    time.sleep(0.1)
            #print coordinate values on gui
#                printCoordsGui(self,PixCoordX,PixCoordY)

            # if the 'q' key is pressed, stop the loop
            if key == ord("r"):

                break

            # Update fps counter
            fps.update()

        # stop timer and disp. fps information
        fps.stop()
        print("Elapsed time: {:.2f}".format(fps.elapsed()))
        print("Approx. FPS: {:.2f}".format(fps.fps()))

        #writing pixel positions, time and radius in an array
        writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray)

        # if we are not using a video file, stop the camera video stream
        if not args.get("video", False):
            vs.stop()
        # otherwise, release the camera
        else:
            vs.release()
        # close all windows
        ser.close()
        cv2.destroyAllWindows()


    def modeMovementUpDown(self):

        time.sleep(0.2)

        try:
             dataByte1, dataByte2, dataByte3 = defaultLookUp()
        except:
            #read in values from look-up table
            dataByte1, dataByte2, dataByte3 = defLookUpTable()

#            byte1 = dataByte1[3]
#            byte2 = dataByte2[3]
#            byte3 = dataByte3[3]
        stepIter = 0

#                time.sleep(0.1)

#                global ser
        ser = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Serial connection with FPGA established ...")
        time.sleep(0.2)
        stepNum = 720
        steps = 719
#                byte1 = 0

        try:
            frameWidth = int(globFrameWidth)
#                print("Frame width manually changed to: ", frameWidth)
        except:
            frameWidth = 720
            print("Default frame width : ", frameWidth)

#            deltaX = 400 #x-offset for centering image coordinate system
#            deltaY = 300 #y-offset for centering image coordinate system

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        args = vars(ap.parse_args())
        buffer = 120
        # define the lower and upper boundaries of the "black"

        try:
            blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
            print("Costumized HSV values entered ...")
        except:
            blackUpper = (100, 90, 90)
            print("Set HSV default values ")
        blackLower = (0, 0, 0)
        pts = deque(maxlen=buffer)

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not args.get("video", False):
            print("Starting video stream...")
#            vs = VideoStream(src=0).start()
            #-threaded videostream
            vs = WebcamVideoStream(src=0).start()
        # otherwise, grab a reference to the video file
        else:
            print(" No video stream possible")
        # allow the camera or video file to warm up
        time.sleep(0.5)
        tracker = None
        writer = None
        confdef = 0.2
        fps = FPSOutput().start()

        coordArrayX = np.array([])
        coordArrayY = np.array([])
        radiusArray = np.array([])
        timeArray = np.array([])
            # keep looping
#                while True:
        while stepIter < 719:

            frame = vs.read()

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                print("No video preview possible")
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=frameWidth)
            height, width, channels = frame.shape
            deltaX = width/2 #x-offset for centering image coordinate system
            deltaY = height/2 #y-offset for centering image coordinate system
#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#            grayscale = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#
            # construct a mask for the color "black", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
        #    mask = cv2.inRange(hsv, greenLower, greenUpper)
        #    mask = cv2.erode(mask, None, iterations=2)
        #    mask = cv2.dilate(mask, None, iterations=2)

            # mask for black color
            mask1 = cv2.inRange(blurred, blackLower, blackUpper)
            mask2 = cv2.erode(mask1, None, iterations=2)

            #Apply median filter
            mask3 = cv2.dilate(mask2, None, iterations=1)
            mask4 = cv2.medianBlur(mask3,5)

            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

#            stopTimer()
            print("Contour length: ",len(cnts))
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if int(M["m00"]) == 0:
                    center = np.nan
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    radius = np.nan
                else:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    PixCoordX = (center[0]-deltaX)
                    PixCoordY = (center[1]-deltaY)*(-1)
                    radius = radius
                print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))

                # only proceed if the radius meets a minimum size
                if radius > 0.01:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

#                    elif radius < 20:

            # update the points queue
                try:
                    print("Contour radius: {:.2f}".format(radius))
                    PixRadius = radius
                except:
                    print("No radius detected")
                    PixCoordX = np.nan
                    PixCoordX = np.nan
            else:
                PixCoordX = np.nan
                PixCoordY = np.nan
                radius = np.nan
                PixRadius = radius
                print("PiX coordinate !!!!!!: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
                print("Contour radius: {:.2f}".format(radius))

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

            coordArrayX = np.append(coordArrayX,(PixCoordX))
            coordArrayY = np.append(coordArrayY,(PixCoordY))
            radiusArray = np.append(radiusArray,(PixRadius))
            timeArray = np.append(timeArray,time.time()) # time in seconds

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF


#                if self.modeChoice.get() == 0:
#                    funcMode1(serialObject)
#
#                if self.modeChoice.get() == 1:
#                    funcMode2(serialObject)
#
#                if self.modeChoice.get() == 2:
#                    funcMode3(serialObject,dataByte1,dataByte2,dataByte3)

            #set serial default 0 position values to fpga
            byte1 = dataByte1[stepIter]
            byte2 = dataByte2[stepIter]
            byte3 = dataByte3[stepIter]
            values = bytearray([byte1, byte2, byte3])
            print("Byte1: ",byte1)
            print("Byte2: ",byte2)
            print("Byte3: ",byte3)
#                ser = serialObject
            ser.write(values)
            print("Data bits written ...")
            stepIter = stepIter + 1
            print("Iteration step: ",stepIter)
#                    time.sleep(0.1)
            #print coordinate values on gui
#                printCoordsGui(self,PixCoordX,PixCoordY)

            # if the 'q' key is pressed, stop the loop
            if key == ord("r"):

                break

            # Update fps counter
            fps.update()

        # stop timer and disp. fps information
        fps.stop()
        print("Elapsed time: {:.2f}".format(fps.elapsed()))
        print("Approx. FPS: {:.2f}".format(fps.fps()))

        #writing pixel positions, time and radius in an array
        writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray)

        # if we are not using a video file, stop the camera video stream
        if not args.get("video", False):
            vs.stop()
        # otherwise, release the camera
        else:
            vs.release()
        # close all windows
        ser.close()
        cv2.destroyAllWindows()