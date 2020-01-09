"""
Created on 26.09.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System
Gui for TinyLev optical particle measurement system

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
from Library import PID
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
from Library.modeMovement import modeMovement



#####################################
#Initialization
#####################################

plt.ion()
x = []
y = []
timer = None
camdistance = 25 # distance from camera lens to central axis of tiny lev
size = 100


x_vec = np.linspace(0,1,size+1)[0:-1]
y_vec = np.random.randn(len(x_vec))
line1 = []
cameraChoice = [
    ("PiCamera"),
    ("USB Camera"),
    ("Stereo Camera"),
    ("USB Camera PID")]

modeMeasuring = [
        ("Mode 1"),
        ("Mode 2"),
        ("Mode 3")]



#Import all essential global variables
global PiCorrdX
global PiCoordY
global selected
global listValue
global selectedCam

global valueHdef
global valueSdef
global valueVdef


valueHdef = 100
valueSdef = 90
valueVdef = 90
#window = tk.Tk()



def setObjPos(spinBoxVal):
    print("Object position changed manually to: ",spinBoxVal)
    global globObjPos
    globObjPos = int(spinBoxVal)
    return globObjPos


def safeSetPoint(self):
    print(self.spinBoxZ.get())


def clickedBack():
    self.destroy()

def connectFPGA():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application."""
    global serialObject
    try:
        print("Establishing connection to FPGA ..")
        serialObject = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Connection established ..")
    except:
        print("Cant Open Specified Port")


def connectFPGACL():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application."""
    global serialObject
    try:
        print("Establishing connection to FPGA ..")
        serialObject = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Connection established ..")
    except:
        print("Cant Open Specified Port")


def closeFPGACL():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application."""

    try:
        print("Closing connection to FPGA ..")
        serialObject.close()
        print("Connection closed ..")
    except:
        print("Cant Open Specified Port")


def funcOpenLoop(listValue):
    global serialObject
    ser = serialObject
    listValue = int(listValue)
    byte1 = dataByte1[listValue]
    byte2 = dataByte2[listValue]
    byte3 = dataByte3[listValue]
    time.sleep(0.05)
    values = bytearray([byte1, byte2, byte3])
    ser.write(values)


def funcClosedLoop(listValue):
    global serialObject
    ser = serialObject
    listValue = int(listValue)
    byte1 = dataByte1[listValue]
    byte2 = dataByte2[listValue]
    byte3 = dataByte3[listValue]
    time.sleep(0.05)
    values = bytearray([byte1, byte2, byte3])
    ser.write(values)


def browseLookUp():
    global data
    global dataByte1
    global dataByte2
    global dataByte3
    print("Choose Look-Up table ..")
    filePath = filedialog.askopenfilename()
    if filePath is None:
        return
    print("Look-Up table imported ..")

    with open(filePath, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(int)
        framemax = len(data[:,1])
        dataByte1 = data[:,0]
        dataByte2 = data[:,1]
        dataByte3 = data[:,2]
        print("Data bytes initialized ..")



def ArduinoConnection():
    if selected == 1:
        global serPC
        print("Establishing serial connection between Computer and Arduino ...")
        try:
            serPC = serial.Serial('COM7', baudrate = 115200)
            print("Connection established!")
            return serPC
        except:
            print("Connection to Arduino failed")
    if selected == 0:
#def ArduinoConnectionRasp():
        global serRasp
        print("Establishing serial connection between Raspberry Pi CM and Arduino ...")
        try:
            serRasp = serial.Serial('/dev/ttyUSB0',115200)
            print("Connection established!")
        except:
            print("Connection to Arduino failed")


def ArduinoConnectionZControl():
    if selected == 1:
        global serPC
        print("Establishing serial connection between Computer and Arduino ...")
        try:
#            serPC = serial.Serial('COM7', baudrate = 57600)
            def writeSetPoint():
                print("ongoing ...")
                ardu= serial.Serial('COM7',115200, timeout=.1)
                time.sleep(1)
                ardu.write('s'.encode())
                time.sleep(1)
                #ardu.close()
            writeSetPoint()
            print("Connection established!")
            return serPC
        except:
            print("Connection to Arduino failed")

    if selected == 0:
        global serRasp
        print("Establishing serial connection between Raspberry Pi CM and Arduino ...")
        try:
            serRasp = serial.Serial('/dev/ttyUSB0',115200)
            print("Connection established!")
            def writeSetPoint():
                print("sent")
                ardu= serial.Serial('/dev/ttyACM0',115200, timeout=.1)
                time.sleep(1)
                ardu.write('s'.encode())
                time.sleep(1)
                #ardu.close()
            writeSetPoint()
        except:
            print("Connection to Arduino failed")


def live_plotter(x_vec,y1_data,line1,identifier='',pause_time=0.1):
    if line1==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)
        #update plot label/title
        plt.ylabel('Y Label')
        plt.title('Title: {}'.format(identifier))
        plt.show()


    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)

    # return line so we can update it again in the next iteration
    return line1

    rand_val = np.random.randn(1)
    y_vec[-1] = rand_val
    line1 = live_plotter(x_vec,y_vec,line1)
    y_vec = np.append(y_vec[1:],0.0)


def showOrbitFile():
#    filepath = filedialog.askopenfilename()
    showOrbit()

def showEMPDFile():
    filepath = filedialog.askopenfilename()
    Calculation.meanParticleSize(filepath)

def printCoordsGui(self, CoordR, CoordZ):
    formatZ = format(CoordZ)
    formatR = format(CoordR)
    self.zCoordLbl['text'] = "Z-Coordinate:  " + formatZ
    self.rCoordLbl['text'] = "R-Coordinate:  " + formatR


def clickedMeanDia():

    selectedCam = selected
    Calculation.meanParticleSize(selectedCam)

def clickedShowRadiusvsTime():
        showRadiusvsFrame()



def posvsTime():
    showPosvsTime()


def safeSetPoint():
    print(spinBoxSetPoint.get())

def setPointTransfer():
    global setPointArduino
    setPointArduino = spinBoxSetPoint.get()

def stopmeasurement():
    FPGAConnection()

def exitCalc():
    exitCalcValue = 1
    print("Exiting ..")

def closeAll():
    print("Exiting ...")

    window.protocol()
    window.destroy()

def binNumber(PixCoordX):
    print("{0:b}".format(PixCoordX))

def clicked():

    selectedCam = selected.get()
    print(selectedCam)



#######################################################
#### Picamera setup
#######################################################

class objectDetection():
#     def __init__(self, name):
#        self.name = name
#        self.tricks = []

    def objectDetectionPicamera(self,selected,reticleIncl):

        if selected == 0:
#            print("Your Selection: PiCam")
#            ArduinoConnection()
            time.sleep(0.2)

    #    if picamerainst == 1:
            # initialize the camera and grab a reference to the raw camera capture
    #            camera = PiCamera()
    #            camera.resolution = (320, 240)
    #            camera.framerate = 90
    #            rawCapture = PiRGBArray(camera, size=(320, 240))

            print("Starting video stream...")

            # allow the camera to warmup
            time.sleep(1.0)
            buffer = 120
            # define the lower and upper boundaries of the "black"
            # ball in the HSV color space, then initialize the
            # list of tracked points
            #blackLower = (29, 86, 6)
            #blackUpper = (64, 255, 25)

            # set default hsv values
            valueHdef = 100
            valueSdef = 90
            valueVdef = 90
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (100, 90, 90)
                print("Set HSV default values ")
#            blackUpper = (50, 50, 100)
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)
            # allow the camera or video file to warm up
            time.sleep(1.0)
            tracker = None
            writer = None
            confdef = 0.4

            print("Sampling THREADED frames from `picamera` module...")
            vs = PiVideoStream().start()
            time.sleep(1.0)
            fps = FPS().start()

            coordArrayX = np.array([])
            coordArrayY = np.array([])
            radiusArray = np.array([])
            timeArray = np.array([])

            # capture frames from the camera
    #            while fps._numFrames < 1000:
            while True:
    #            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
    #                frame = frame.array
    #                frame = PiVideoStream().start()
    #                frame = imutils.resize(frame, width = 400)

                frame = vs.read()
                try:
                    frame.shape
                    print("checked for shape".format(frame.shape))
                except AttributeError:
                    print("shape not found")
                    #code to move to next frame


                frame = imutils.resize(frame, width = 640)

                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

#                mask1 = cv2.inRange(hsv,cvS)

                mask1 = cv2.inRange(hsv, blackLower, blackUpper)
                mask2 = cv2.erode(mask1, None, iterations=1)
                mask3 = cv2.dilate(mask2, None, iterations=1)


                #Apply median filter
    #            mask3 = cv2.dilate(mask2, None, iterations=1)
    #                mask4 = cv2.medianBlur(mask2,5)


                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                center = None

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
                        PixCoordX = (center[0])
                        PixCoordY = (center[1])
                        radius = radius
                    print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
            #                pixcoordinate
#                    PCoordXBin = bin(PixCoordX)
#                    PCorrdYBin = bin(PixCoordY)
                    PCoordXBin = PixCoordX
                    PCorrdYBin = PixCoordY
                    # write data to arduino
#                try:
#                    writeDataArduino(PCoordXBin, PCorrdYBin)
#                    writeDataArduino(PCoordXBin)
#                except:
#                    None

                    # only proceed if the radius meets a minimum size
                    if radius > 0.01:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius))
                        PixRadius = radius
        #                    serRasp = serial.Serial('/dev/ttyUSB0',57600)
        #                    serRasp.write(PixCoordX)
        #                    serRasp.write(PixCoordY)
        #                    writeDataArduino(PixCoordX,PixCoordY)
                        pts.appendleft(center)
                    except:
                        print("No radius detected ...")
                        PixCoordX = np.nan
                        PixCoordX = np.nan

                else:
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    radius = np.nan
                    PixRadius = radius
                    print("PiX coordinate : {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
                    print("Contour radius: {:.2f}".format(radius))

                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                try:
                    coordArrayX = np.append(coordArrayX,abs(PixCoordX))
                    coordArrayY = np.append(coordArrayY,abs(PixCoordY))
                    radiusArray = np.append(radiusArray,abs(PixRadius))
                    timeArray = np.append(timeArray,time.time())

                except:print("No radius detected ...")

                # show the frame
                if reticleIncl == 1:
                    height, width, channels = frame.shape
                    frame = frame.copy()
                    cv2.circle(frame, (int(width/2), int(height/2)), 10, (255, 0, 0), -1)
                    cv2.line(frame, (int(width/2-100), int(height/2)), (int(width/2+100), int(height/2)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frame, (int(width/2), int(height/2-100)), (int(width/2), int(height/2+100)),(255, 0, 255), 4) #x1,y1,x2,y2
#                cv2.imshow("mask1", mask1)
                cv2.imshow("frame", frame)
#                cv2.imshow("mask2", mask2)
                key = cv2.waitKey(1) & 0xFF


                if keyboard.is_pressed('r'):  # if key 'r' is pressed
                    print('Exiting ...')
                    break  # finishing the loop

                fps.update()

            fps.stop()
            print("Elapsed time: {:.2f}".format(fps.elapsed()))
            print("Approx. FPS: {:.2f}".format(fps.fps()))

            writePixelPositionRasp(timeArray,coordArrayX,coordArrayY,radiusArray)

            cv2.destroyAllWindows()
            vs.stop()



####################################################
#### USB camera setup
####################################################

    def objectDetectionUSBCamera(self,selected,reticleIncl):


        if selected == 1:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 720
                print("Default frame width: ", frameWidth)

            try:
                objDia = int(globObjDia)
            except:
                objDia = 2 #in mm
                print("Default object diameter: ", objDia)

            P = 0.5
            I = 1.5
            D = 0.3
            posZ = 0
            initPIDParams(posZ,P,I,D)
            time.sleep(0.1)
            createConfigPID()
            time.sleep(0.1)

            try:
                dataByte1, dataByte2, dataByte3 = defaultLookUp()
            except:
            #read in values from look-up table
                dataByte1, dataByte2, dataByte3 = defLookUpTable()

            # construct the argument parse and parse the arguments
            ap = argparse.ArgumentParser()
            args = vars(ap.parse_args())
            buffer = 120
            # define the lower and upper boundaries of the "black"
            # ball in the HSV color space, then initialize the
#            deltaX = 300 #x-offset for centering image coordinate system
#            deltaY = 400 #y-offset for centering image coordinate system
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
            time.sleep(0.2)
            tracker = None
            writer = None
            confdef = 0.2
            framenum = 0
            fps = FPSOutput().start()
            coordArrayX = np.array([])
            coordArrayY = np.array([])
            radiusArray = np.array([])
            timeArray = np.array([])
            tempFrames = np.array([])
            tempPartDiaPixels = np.array([])
            pidOutputArray = np.array([])


            # keep looping
            while True:

    #            startTimer()
                # grab the current frame
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
                        pixDiameter = 2*radius
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
                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
                    print("Contour radius: {:.2f}".format(radius))

                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
    #            cv2.imshow('HSV', mask1)
    #            cv2.imshow('Erode', mask2)
    #            cv2.imshow('Dilate', mask3)
    #            cv2.imshow("Median Filter", mask4)
                if reticleIncl == 1:
                    height, width, channels = frame.shape
                    frame = frame.copy()
                    cv2.circle(frame, (int(width/2), int(height/2)), 10, (255, 0, 0), -1)
                    cv2.line(frame, (int(width/2-100), int(height/2)), (int(width/2+100), int(height/2)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frame, (int(width/2), int(height/2-100)), (int(width/2), int(height/2+100)),(255, 0, 255), 4) #x1,y1,x2,y2

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF


                pid = readConfigPID()
                pid.update(PixCoordY)
                pidOutputVal = float(pid.output)
#                print("PID output: ",pid.output)
                # if the 'q' key is pressed, stop the loop
                if key == ord("r"):
    #                cmdStopTimer()
                    break
                elif selectedStopAll == 1:
                    break

#                try:
#                    spinBoxVal = int(spinBoxVal)
#                except:
#                    spinBoxVal = 0 #in mm

                objZPos = setObjPos(spinBoxVal)
                print("Obj Z position value: ",objZPos)
                byte1 = dataByte1[int(objZPos)]
                byte2 = dataByte2[int(objZPos)]
                byte3 = dataByte3[int(objZPos)]
                values = bytearray([byte1, byte2, byte3])
                serialObject.write(values)

                print("Serial Values: ",byte1)

                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                try:
                    tempPartDiaPixels = np.append(tempPartDiaPixels,pixDiameter)
                except:
                    None
                pidOutputArray = np.append(pidOutputArray,pidOutputVal)
                coordArrayX = np.append(coordArrayX,abs(PixCoordX))
                coordArrayY = np.append(coordArrayY,abs(PixCoordY))
                radiusArray = np.append(radiusArray,abs(PixRadius))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))

                window.update()

#                time.sleep(0.5)
                # Update fps counter
                fps.update()


            # stop timer and disp. fps information
            fps.stop()
            fpsVar = float((fps.fps()))
            print("Elapsed time: {:.2f}".format(fps.elapsed()))
            print("Approx. FPS: {:.2f}".format(fps.fps()))

            #calculate mean diameter
            meanPartDia = sum(tempPartDiaPixels)/frameCounter
            print("Mean particle diameter / px : {:.2f}".format(meanPartDia))

            # transform from px to mm
            meanPartDiamm = meanPartDia/objDia
            print("Particle resolution / px/mm : {:.2f}".format(meanPartDiamm))

            # time per frame
            meanTimeFrame = fps.elapsed()/framenum
            print("Frame mean time / s: {:.2f}".format(meanTimeFrame))
#            PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, fps.elapsed())

            writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray,framenum,fpsVar)
            writePIDOutput(timeArray,coordArrayY,pidOutputArray)

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs.stop()

            # otherwise, release the camera
            else:
                vs.release()

            # close all windows
            cv2.destroyAllWindows()

#    def _objectDetectionUSBCamera(self,selected,reticleIncl):
#
#        threading.Thread(target=self.objectDetectionUSBCamera).start()


####################################################
#### Stereo Vision
####################################################


    def objectDetectionOther(self,selected,reticleIncl):

        if selected == 2:

            print("Starting up cameras ...")
            time.sleep(0.5)

            video_capture_0 = cv2.VideoCapture(0)
            video_capture_1 = cv2.VideoCapture(1)

            while True:
                # Capture frame-by-frame
                ret0, frame0 = video_capture_0.read()
                ret1, frame1 = video_capture_1.read()

                # define size of image
                frame0 = imutils.resize(frame0, width = 1024)
                frame1 = imutils.resize(frame1, width = 1024)

                if (ret0):
                    # Display the resulting frame
                    cv2.imshow('Cam 0', frame0)

                if (ret1):
                    # Display the resulting frame
                    cv2.imshow('Cam 1', frame1)

                if cv2.waitKey(1) & 0xFF == ord('r'):
                    break

            # When everything is done, release the capture
            video_capture_0.release()
            video_capture_1.release()
            cv2.destroyAllWindows()


####################################################
#### Z-Position measurement mode 3
####################################################

    def objectDetectionZMeasurement(self,selected,reticleIncl):

        if selected == 3:

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

                if reticleIncl == 1:
                    height, width, channels = frame.shape
                    frame = frame.copy()
                    cv2.circle(frame, (int(width/2), int(height/2)), 10, (255, 0, 0), -1)
                    cv2.line(frame, (int(width/2-100), int(height/2)), (int(width/2+100), int(height/2)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frame, (int(width/2), int(height/2-100)), (int(width/2), int(height/2+100)),(255, 0, 255), 4) #x1,y1,x2,y2

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF


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
    #                cmdStopTimer()
                    break
                elif selectedStopAll == 1:
                    break
                window.update()
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



class GUI():

    def __init__(self, master,cameraChoice, *args, **kwargs):
#        Frame.__init__(self, master, *args, **kwargs)
        super().__init__(*args, **kwargs)
        self.master = master
#        self.master = tk.Toplevel(self.master)
#        self.frame = Frame(self.master)
        self.master.title("TinyLev")
        self.master.geometry("900x650+1000+600")


#        selected = tk.IntVar()
        self.cameraChoice = cameraChoice
        self.choice = tk.IntVar()
        self.label1 = tk.Label(self.master, text = "Select Camera").grid(row=0, sticky=tk.W, columnspan=3)


        for index, cameraChoice in enumerate(cameraChoice):
                    b = tk.Radiobutton(self.master, text=cameraChoice, variable=self.choice,
                                    value=index, command = self.changeValue)
                    b.grid(row=index+1, column=1,sticky = tk.W)


        self.b1 = Button(self.master, text = "Start", command = self.setValue)
        self.b1.grid(column=7, row = 2,sticky = tk.W+tk.E,columnspan =1)

        self.b9 = Button(self.master, text="Stop ", command = self.stopAll)
        self.b9.grid(column=8,row=2,sticky = tk.W+tk.E,columnspan =1)

#        self.b2 = Button(self.master, text = "Stop Measurement", command = stopmeasurement)
#        self.b2.grid(column= 7, row = 4,sticky = tk.W+tk.E,columnspan =1 )

        self.b3 = Button(self.master, text = "Show Orbit", command = showOrbitFile)
        self.b3.grid(column= 7, row = 6,sticky = tk.W+tk.E, columnspan =1)

        self.b4 = Button(self.master, text = "Show Radius Distribution", command = clickedShowRadiusvsTime)
        self.b4.grid(column= 7, row = 7,sticky = tk.W+tk.E, columnspan =1)

        self.b5 = Button(self.master, text = "Estimate Mean Patricle Diameter", command = clickedMeanDia)
        self.b5.grid(column= 8, row = 4,sticky = tk.W+tk.E, columnspan =1)

        self.b6 = Button(self.master, text = "Position-Time", command = posvsTime)
        self.b6.grid(column= 8, row = 5,sticky = tk.W+tk.E, columnspan =1)

        self.b7 = Button(self.master, text = "Z-Closed-Loop", command = self.newWinZCL)
        self.b7.grid(column= 8, row = 6,sticky = tk.W+tk.E, columnspan =1)

        self.b8 = Button(self.master, text = "Measuring Z-Position", command = self.newWinZOL)
        self.b8.grid(column= 8, row = 7,sticky = tk.W+tk.E, columnspan =1)

        self.b9 = Button(self.master, text = "Show FFT", command = self.showFFTfcn)
        self.b9.grid(column= 8, row = 8,sticky = tk.W+tk.E, columnspan =1)

        self.setPnt = tk.Label(self.master, text="H-Value: ")
        self.setPnt.grid(column=2,row=10,sticky = tk.W+tk.E)

        self.spinBoxH = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueH)
        self.spinBoxH.grid(column=7, row= 10,sticky = tk.W+tk.E)

        self.setPnt = tk.Label(self.master, text="S-Value: ")
        self.setPnt.grid(column=2,row=11,sticky = tk.W+tk.E)
                # Create a spinbox for S
        self.spinBoxS = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueS)
        self.spinBoxS.grid(column=7, row= 11,sticky = tk.W+tk.E)

        self.setPnt = tk.Label(self.master, text="V-Value: ")
        self.setPnt.grid(column=2,row=12,sticky = tk.W+tk.E)

        self.spinBoxV = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueV)
        self.spinBoxV.grid(column=7, row= 12,sticky = tk.W+tk.E)

        self.b10 = Button(self.master, text="Set Values ", command = self.setAllValuesHSV)
        self.b10.grid(column=8,row=11,sticky = tk.W)

        self.b11 = Button(self.master, text="Default ", command = self.setHSVDef)
        self.b11.grid(column=8,row=12,sticky = tk.W)

        self.b12 = Button(self.master, text="Exit ", command = closeAll)
        self.b12.grid(column=8,row=9,sticky = tk.W+tk.E,columnspan =1)

        self.varRetBox = IntVar()
        self.RetBox = Checkbutton(self.master, text = "Include Reticle", variable=self.varRetBox,command=self.setReticleIncl)
        self.RetBox.grid(column=1,row=8,sticky = tk.W+tk.E,columnspan =1)

        self.rCoordLbl = tk.Label(self.master, text="R-Coordinate: --:--")
        self.rCoordLbl.grid(column=2,row=13,sticky = tk.W+tk.E)

        self.zCoordLbl = tk.Label(self.master, text="Z-Coordinate: --:--")
        self.zCoordLbl.grid(column=2,row=14,sticky = tk.W+tk.E)

        self.setPntFrame = tk.Label(self.master, text="Define frame width: ")
        self.setPntFrame.grid(column=2,row=15,sticky = tk.W+tk.E)
                # Create a spinbox for H
        self.spinBoxWidth = tk.Spinbox(self.master, from_ =1, to=1024, command = self.printFrameWidth)
        self.spinBoxWidth.grid(column=7, row= 15,sticky = tk.W+tk.E)

        self.b13 = Button(self.master, text="Set Width ", command = self.setFrameWidth)
        self.b13.grid(column=8,row=15,sticky = tk.W)

        self.setPntFrame = tk.Label(self.master, text="Define object diameter: ")
        self.setPntFrame.grid(column=2,row=16,sticky = tk.W+tk.E)

        self.spinBoxObjDia = tk.Spinbox(self.master, from_ =1, to=1024, command = self.printObjDia)
        self.spinBoxObjDia.grid(column=7, row= 16,sticky = tk.W+tk.E)

        self.b14 = Button(self.master, text="Set diameter ", command = self.setObjDia)
        self.b14.grid(column=8,row=16,sticky = tk.W)

        self.b15 = Button(self.master, text="Plot PID ", command = self.setPIDPlot)
        self.b15.grid(column=7,row=8,sticky = tk.W+tk.E,columnspan =1)

        self.setPosFrame = tk.Label(self.master, text="Define object position: ")
        self.setPosFrame.grid(column=2,row=17,sticky = tk.W+tk.E)

        self.spinBoxObjPos = tk.Spinbox(self.master, from_ =-0, to=720,textvariable = 0, command = self.setObjPosTemp)
        self.spinBoxObjPos.grid(column=7, row= 17,sticky = tk.W+tk.E)

        self.b16 = Button(self.master, text="Set position ", command = self.setObjPosTemp)
        self.b16.grid(column=8,row=17,sticky = tk.W)

        self.b17 = Button(self.master, text = "Connect FPGA", command = connectFPGACL)
        self.b17.grid(column= 7, row = 4, sticky = tk.W+tk.E)

        self.b18 = Button(self.master, text = "Close FPGA", command = closeFPGACL)
        self.b18.grid(column= 7, row = 5, sticky = tk.W+tk.E)



    def printValueH(self):
        print("H-value: {} ".format(self.spinBoxH.get()))

    def printValueS(self):
        print("S-value: {} ".format(self.spinBoxS.get()))

    def printValueV(self):
        print("V-value: {} ".format(self.spinBoxV.get()))

    def setAllValuesHSV(self):
        global valueUpperH
        global valueUpperS
        global valueUpperV
        valueUpperH = self.spinBoxH.get()
        valueUpperS = self.spinBoxS.get()
        valueUpperV = self.spinBoxV.get()
        print("HSV values set")
        print("H",valueUpperH)
        print("S",valueUpperS)
        print("V",valueUpperV)
#        return valueUpperH, valueUpperS, valueUpperV


    def printObjPos(self):
        print("Object position: ", self.spinBoxObjPos.get())


    def printObjDia(self):
        print("Frame width: ", self.spinBoxObjDia.get())

    def setObjPosTemp(self):
        global spinBoxVal
        spinBoxVal = self.spinBoxObjPos.get()
        setObjPos(spinBoxVal)

    def setObjDia(self):
        print("Object diameter changed manually to: ",self.spinBoxObjDia.get())
        global globFrameWidth
        globObjDia = int(self.spinBoxObjDia.get())
        return globObjDia

    def setPIDPlot(self):
        showPIDPlot()


    def printFrameWidth(self):
        print("Frame width: ", self.spinBoxWidth.get())


    def setFrameWidth(self):
        print("Frame width changed manually to: ",self.spinBoxWidth.get())
        global globFrameWidth
        globFrameWidth = int(self.spinBoxWidth.get())
        return globFrameWidth


    def showFFTfcn(self):
        showFFT()


    def setHSVDef(self):
        # set default hsv values
        valueHdef = 100
        valueSdef = 90
        valueVdef = 90
        print("HSV default values set")
        print("H",valueHdef)
        print("S",valueSdef)
        print("V",valueVdef)


    def changeValue(self):
        global selected
        print("Your choice: {} ".format(cameraChoice[self.choice.get()]))
        selected = self.choice.get()

    def onTimer():
        print("Timer started")

    def setValue(self, isRunningFunc=None):
        global selectedStopAll
        selectedStopAll = 0
        try:
            reticleIncl = self.varRetBox.get()
        except:
            reticleIncl = 0
        if selected == 0:
            objectDetection.objectDetectionPicamera(self,selected,reticleIncl)
        elif selected == 1:
            objectDetection.objectDetectionUSBCamera(self,selected,reticleIncl)
        elif selected == 2:
            objectDetection.objectDetectionOther(self,selected,reticleIncl)
        elif selected == 3:
            objectDetection.objectDetectionZMeasurement(self,selected,reticleIncl)


    def setReticleIncl(self):
        global reticleIncl
        print("Reticle value:",self.varRetBox.get()) #if checkbox is activated, value is equak to 1, otherwise 0
        reticleIncl = self.varRetBox.get()


    def stopAll(self):
        global selectedStopAll
        selectedStopAll = 1

    def clickedMeanDia():
        PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, fps.elapsed())
#        global selectedCam
#        selectedCam = selected.get()
#        Calculation.meanParticleSize(selectedCam)


    def newWinZOL(self):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        newBtnZOL = measureZPosOL(self.newWindow,modeMeasuring)


    def newWinZCL(self):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        newBtnZCL = winZClosedLoop(self.newWindow)

class measureZPosOL():

    def __init__(self, master, modeMeasuring, *args, **kwargs):

        super().__init__(*args, **kwargs)
        self.master = master
        self.frame = Frame(self.master)
        self.master.lift()

        self.master.title("Measuring Z-Position")
        self.master.geometry('600x400+1000+600')


        self.modeChoice = tk.IntVar()
        self.modeMeasuring = modeMeasuring
        selected = tk.IntVar()

        self.stpmsm = tk.Button(self.master, text = "FPGA Set-Up", command = connectFPGA)
#        self.stpmsm.grid(column= 3, row = 4, sticky = tk.W+tk.E,columnspan = 1)
        self.stpmsm.grid(column=3,row=1,sticky = tk.W)

        self.shworb = tk.Button(self.master, text = "Browse", command = browseLookUp)
#        self.shworb.grid(column= 3, row = 5, sticky =tk.W+tk.E,columnspan = 1)
        self.shworb.grid(column=3,row=2,sticky = tk.W)

        self.bck = tk.Button(self.master, text = "Back", command = self.mainGUI)
#        self.bck.grid(column= 3, row = 6, sticky = tk.W+tk.E,columnspan = 1)
        self.bck.grid(column=6,row=1,sticky = tk.W)

        # Create a spinbox
        self.spinBoxZ = tk.Spinbox(self.master, from_ =0, to=719, command = self.printValues)
#        self.spinBoxZ.grid(column=3, row= 9,sticky = tk.W+tk.E)
        self.spinBoxZ.grid(column=3,row=7,sticky = tk.W)

        self.setBtn = tk.Button(self.master, text = "Set", command = self.printValues)
#        self.setBtn.grid(column=4, row = 9,sticky = tk.W+tk.E, columnspan = 1)
        self.setBtn.grid(column=4,row=7,sticky = tk.W)

        self.setBtn = tk.Button(self.master, text = "Start", command = self.startMeasZPosOL)
#        self.setBtn.grid(column=4, row = 9,sticky = tk.W+tk.E, columnspan = 1)
        self.setBtn.grid(column=5,row=7,sticky = tk.W)

        self.resetBtn = tk.Button(self.master, text = "Reset", command = self.resetBytes)
#        self.setBtn.grid(column=4, row = 9,sticky = tk.W+tk.E, columnspan = 1)
        self.resetBtn.grid(column=6,row=7,sticky = tk.W)

        self.setPnt = tk.Label(self.master, text="Set point: ")
#        self.setPnt.grid(column=2,row=9,sticky = tk.W+tk.E)
        self.setPnt.grid(column=3,row=8,sticky = tk.W)

        self.label1 = tk.Label(self.master, text = "Select Mode").grid(row=0, sticky=tk.W, columnspan=1)


        for indexMode, modeMeasuring in enumerate(modeMeasuring):
                    b = tk.Radiobutton(self.master, text=modeMeasuring, variable=self.modeChoice,
                                    value=indexMode, command = self.modeChangeValue)
                    b.grid(row=indexMode+1, column=0,sticky = tk.W)
#


    def mainGUI(self):
#        window.destroy()
        self.master.withdraw()
        self.master = tk.Toplevel(self.master)
        bckBtn = GUI(self.master,cameraChoice)


    def modeChangeValue(self):
        global selectedMode
        global dataByte1
        global dataByte2
        global dataByte3
        print("Your choice: {} ".format(modeMeasuring[self.modeChoice.get()]))
        selectedMode = self.modeChoice.get()
        dataByte1, dataByte2, dataByte3 = defaultLookUp()


    def printValues(self):
        print("List value is: ", self.spinBoxZ.get())
        listValue = self.spinBoxZ.get()
        funcOpenLoop(listValue)


    def printScaleVal(self):
#        print("Scaler value: {} ".format(self.setScaler.get()))
        print("Scaler value: {} ")


    def destroyWind(self):
        window.destroy()
        sys.exit()


    def resetBytes(self):
        ser = serial.Serial('COM12', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        with open("./Data/LookUpTable_0_720_Dec.csv", 'r') as f:
            reader = csv.reader(f, delimiter=';')
            # get header from first row
            headers = next(reader)
            # get all the rows as a list
            data = list(reader)
            # transform data into numpy array
            data = np.array(data).astype(int)
            framemax = len(data[:,1])
            dataByte1 = data[:,0]
            dataByte2 = data[:,1]
            dataByte3 = data[:,2]
            print("Default look-up table loaded ...")
        byte1 = dataByte1[0]
        byte2 = dataByte2[0]
        byte3 = dataByte3[0]
        values = bytearray([byte1, byte2, byte3])
        ser.write(values)
        ser.close()
        print("Serial bytes changed to default values ...")


    def startMeasZPosOL(self):
        if self.modeChoice.get() == 0:
            modeMovement.modeMovementUp(self)
        elif self.modeChoice.get() == 1:
            modeMovement.modeMovementDown(self)
        elif self.modeChoice.get() == 2:
            modeMovement.modeMovementUpDown(self)


class winZClosedLoop():

    def __init__(self, master):


        self.master = master
        self.frame = Frame(self.master)
        self.master.lift()
        self.master.title("Z-Control")
#        mainWin = Toplevel(window)
        self.master.geometry('800x400+1000+600')
#        Style().configure("TButton", padding=(0, 5, 0, 5),
#            font='serif 10')
#
        selected = tk.IntVar()

        self.stpmsm = tk.Button(self.master, text = "FPGA Set-Up", command = connectFPGA)
#        self.stpmsm.grid(column= 3, row = 4, sticky = tk.W+tk.E,columnspan = 1)
        self.stpmsm.pack()

        self.shworb = tk.Button(self.master, text = "Browse", command = browseLookUp)
#        self.shworb.grid(column= 3, row = 5, sticky =tk.W+tk.E,columnspan = 1)
        self.shworb.pack()

        self.bck = tk.Button(self.master, text = "Back", command = self.mainGUI)
#        self.bck.grid(column= 3, row = 6, sticky = tk.W+tk.E,columnspan = 1)
        self.bck.pack()

        # Create a spinbox
        self.spinBoxZ = tk.Spinbox(self.master, from_ =0, to=720, command = self.printValuesCL)
#        self.spinBoxZ.grid(column=3, row= 9,sticky = tk.W+tk.E)
        self.spinBoxZ.pack()

        self.setBtn = tk.Button(self.master, text = "Set", command = self.printValuesCL)
#        self.setBtn.grid(column=4, row = 9,sticky = tk.W+tk.E, columnspan = 1)
        self.setBtn.pack()

        self.setPnt = tk.Label(self.master, text="Set point: ")
#        self.setPnt.grid(column=2,row=9,sticky = tk.W+tk.E)
        self.setPnt.pack()

#        self.setBtn = tk.Button(self.master, text = "Exit", command = self.destroyWind)
#        self.setBtn.grid(column=7, row = 11,sticky = tk.W+tk.E, columnspan = 1)

#        self.setScaler = tk.Scale(self.master, from_=0, to = 720, orient=HORIZONTAL,length=500,command = self.printScaleVal,
#                                  )
##        self.setScaler.grid(column=9, row = 9,padx = 100, columnspan = 1)
#        self.setScaler.pack()
#


    def mainGUI(self):
        self.master.withdraw()
        self.master = tk.Toplevel(self.master)
        bckBtn = GUI(self.master,cameraChoice)


    def printValuesCL(self):
        print("List value is: ", self.spinBoxZ.get())
        listValue = self.spinBoxZ.get()
        funcClosedLoop(listValue)

    def printScaleVal(self):
#        print("Scaler value: {} ".format(self.setScaler.get()))
        print("Scaler value: {} ")

    def destroyWind(self):
        window.destroy()
        sys.exit()

key = cv2.waitKey(1) & 0xFF

if key == ord("r"):
    self.tk.mainloop(n)

if __name__ == '__main__':
    window = tk.Tk()
    app = GUI(window,cameraChoice)
    window.mainloop()
    window.update()
