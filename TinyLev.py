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
from Library import PIDMultiX
from Library import PIDMultiY
from Library import PIDMultiZ
from Library.stereoCameraTest import *
from Library.stereoChessRecognition import *
from Library.Calibration3D import *
from Library.depthMapTuning import *
from Library.liveDepthMap import *
from Library.depthMapTuning import *
from Library.MultiLev import *
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
import time
import threading
import queue
import sys
import random
import serial
import pandas as pd
import math
from PIL import Image
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
    ("USB Camera PID"),
    ("Stereo Camera"),
    ("Stereo Cropped"),
    ("MultiLev")]

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




def setObjPos(self, spinBoxVal):
    print("Object position changed manually to: ",spinBoxValX)
    self.globObjPos


def setObjPosX(self, spinBoxValX):
#    print("Object position changed manually to: ", spinBoxValX)
    self.globObjPosX = int(spinBoxValX)
    return self.globObjPosX


def setObjPosY(self, spinBoxValY):
#    print("Object position changed manually to: ", spinBoxValY)
    self.globObjPosY = int(spinBoxValY)
    return self.globObjPosY


def setObjPosZ(self, spinBoxValZ):
#    print("Object position changed manually to: ", spinBoxValZ)
    self.globObjPosZ = int(spinBoxValZ)
    return self.globObjPosZ


def setObjDiamm(self, objDia):
    print("Object diameter changed manually to: ",objDia)
    self.globObjDia = int(self.objDia)
    return self.globObjPos

def safeSetPoint(self):
    print(self.spinBoxZ.get())


def clickedBack(self):
    self.destroy()

def connectFPGA():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application."""
    global serialObject
    try:
        print("Establishing connection to FPGA ..")
        serialObject = serial.Serial('COM12', 250000, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        print("Connection established ..")
    except:
        print("Cant Open Specified Port")


def connectFPGACL():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application."""
    global serialObject
    try:
        print("Establishing connection to FPGA ..")
        serialObject = serial.Serial('COM12', 250000, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
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

def printCoordsGui(self, xTri, yTri, zTri):
    formatX = ("{0:.1f}".format(xTri))
    formatY = ("{0:.1f}".format(yTri))
    formatZ = ("{0:.1f}".format(zTri))

    self.xCoordLbl['text'] = "X-Coordinate:   " + formatX
    self.yCoordLbl['text'] = "Y-Coordinate:   " + formatY
    self.zCoordLbl['text'] = "Z-Coordinate:   " + formatZ


def clickedMeanDia():

    selectedCam = selected
    Calculation.meanParticleSize(selectedCam)


def clickedShowRadiusvsTime():
        showRadiusvsFrame()


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
    window.update()
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

    def objectDetectionPicamera(self,selected,PIDIncl,reticleIncl):

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


            try:
                tempObjDiaReal = setObjDiamm()
            except:
                tempObjDiaReal = 2.1 #mm
            try:
                tempObjDiaPx = setObjDiapx()
            except:
                tempObjDiaPx = 40 #px


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
                    if (int(M["m00"]) != 0) and (radius < 65 and radius > 20):
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
                if self.reticleIncl == 1:
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

    def objectDetectionUSBCamera(self,selected,PIDIncl,reticleIncl):


        if selected == 1:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 1024
                print("Default frame width: ", frameWidth)

            try:
                objDia = int(globObjDia)
            except:
                objDia = 2 #in mm
                print("Default object diameter: ", objDia)

            try:
                tempObjDiaReal = setObjDiamm()
            except:
                tempObjDiaReal = 2.1 #mm
            try:
                tempObjDiaPx = setObjDiapx()
            except:
                tempObjDiaPx = 40 #px

            #Cameras positioning
            alphaCam = 18.4
            updateRatePos = 25
            P = 0.5
            I = 1.5
            D = 0.3
            posZ = 0
            initPIDParams(posZ,P,I,D)
            time.sleep(0.1)
            createConfigPID(posZ,P,I,D)
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
                blackUpper = (120, 90, 90)
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
            deltaWidth = 128
            deltaHeight = 128
            width= 1024
            height = 576
            halfCamWidth = width/2 #x-offset for centering image coordinate system
            halfCamHeight = height/2 #y-offset for centering image coordinate system
            halfCamWidth = int(halfCamWidth)
            halfCamHeight = int(halfCamHeight)
            PixCoordX = 0
            PixCoordY = 0
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
#                frame = imutils.resize(frame, width=frameWidth)
                frameCropped = imutils.resize(frame, width=frameWidth)

#                if frame is not None:
#                    frameCropped[0:576,0:(400+(int(PixCoordX))),:] = 255 #left side bar
#                    frameCropped[188:400,(700+int(PixCoordX)):1024,:] = 255 #right side bar
#                    frameCropped[0:250-int(PixCoordY),0:1024,:] = 255 #upper cross bar
#                    frameCropped[326-int(PixCoordY):576,0:1024,:] = 255 #lower cross bar
#                else:
#                    frameCropped[0:576,0:400,:] = 255 #left side bar
#                    frameCropped[188:400,700:1024,:] = 255 #right side bar
#                    frameCropped[0:200,0:1024,:] = 255 #upper cross bar
#                    frameCropped[376:576,0:1024,:] = 255 #lower cross bar

                frameCropped1 = frameCropped[halfCamHeight-int(deltaHeight):halfCamHeight+int(deltaHeight),halfCamWidth-int(deltaWidth):halfCamWidth+int(deltaWidth),:]
#                height, width, channels = frame.shape
#                deltaX = width/2 #x-offset for centering image coordinate system
#                deltaY = height/2 #y-offset for centering image coordinate system

#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
                blurred = cv2.GaussianBlur(frameCropped1, (11, 11), 0)
#                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
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
                    if (int(M["m00"]) != 0) and (radius < 150 and radius > 20):
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        print("Center[0]",center[0])
                        print("Center[1]",center[1])
#                        PixCoordX1 = (center[0]-deltaWidth)
                        PixCoordX = (center[0]-deltaWidth)*math.cos(alphaCam*(2*math.pi/360))
                        PixCoordY = (center[1]-deltaHeight)*(-1)
                        radius = radius
                        pixDiameter = 2*radius
#                        print("X",PixCoordX1)
#                        print("X-Cos",PixCoordX)
                    else:
                        center = np.nan
                        PixCoordX = np.nan
                        PixCoordY = np.nan
                        radius = np.nan
                    print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))



                    # only proceed if the radius meets a minimum size
                    if (radius > 20 and radius < 150):
#                    if (radius > 0):

                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frameCropped1, (center[0], center[1]), int(radius), (15, 186, 2), 10)
                        cv2.circle(frameCropped1, center, 5, (0, 0, 255), -1)


#                    elif radius < 20:

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius))
                        PixRadius = radius
                    except:
                        print("No radius detected")
                        PixRadius = np.nan
                        PixRadius = np.nan
                else:
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    radius = np.nan
                    PixRadius = radius
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")

                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
#                cv2.imshow('HSV', mask1)
#                cv2.imshow('Erode', mask2)
#                cv2.imshow('Dilate', mask3)
#                cv2.imshow("Median Filter", mask4)
                if self.reticleIncl == 1:
#                    height, width, channels = frame.shape
#                    frame = frame.copy()
#                    cv2.circle(frameCropped, (int(width/2), int(height/2)), 10, (255, 0, 0), -1)
#                    cv2.line(frameCropped, (int(width/2-100), int(height/2)), (int(width/2+100), int(height/2)),(255, 0, 255), 4) #x1,y1,x2,y2
#                    cv2.line(frameCropped, (int(width/2), int(height/2-100)), (int(width/2), int(height/2+100)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.circle(frameCropped1, (int(deltaWidth), int(deltaHeight)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped1, (int(deltaWidth-50), int(deltaHeight)), (int(deltaWidth+50), int(deltaHeight)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped1, (int(deltaWidth), int(deltaHeight-50)), (int(deltaWidth), int(deltaHeight+50)),(255, 0, 255), 4) #x1,y1,x2,y2
                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                if framenum % 3 == 0:
                    cv2.imshow("Frame", frameCropped1)
                    key = cv2.waitKey(1) & 0xFF

                if self.PIDIncl == 1:
                    pid = readConfigPID()
                    pid.SetPoint = float(spinBoxValZ)
                    pid.setSampleTime(0.001)
                    pid.setKp(1.1) #default: 3.1
                    pid.setKi(100) #default: 89.7
                    pid.setKd(0.025) #default: 0.025
#                    pid.update(PixCoordY)
                    pid.update(PixCoordY)
                    pidOutputVal = float(pid.output)
                    print("PID output",pid.output)

                else:
                    pidOutputVal = np.nan

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

                #open-loop control adjusted via the user interface
                objZPosZ = setObjPosZ(self, spinBoxValZ)
                print("Obj Z position value: ",objZPosZ)
                if (objZPosZ >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(objZPosZ)]
                    byte2 = dataByte2[int(objZPosZ)]
                    byte3 = dataByte3[int(objZPosZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)
                elif (objZPosZ < 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(720 + objZPosZ)]
                    byte2 = dataByte2[int(720 + objZPosZ)]
                    byte3 = dataByte3[int(720 + objZPosZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)


#                #closed-loop control adjusted via the user interface
                if (pidOutputVal <= 0 and self.PIDIncl == 1):
                    objZPosCLZ = (pidOutputVal/20)
                    byte1 = dataByte1[int(719 + objZPosCLZ)]
                    byte2 = dataByte2[int(719 + objZPosCLZ)]
                    byte3 = dataByte3[int(719 + objZPosCLZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                elif (pidOutputVal > 0 and self.PIDIncl == 1):
                    objZPosCLZ = (pidOutputVal/20)
                    byte1 = dataByte1[int(objZPosCLZ+objZPosZ)]
                    byte2 = dataByte2[int(objZPosCLZ+objZPosZ)]
                    byte3 = dataByte3[int(objZPosCLZ+objZPosZ)]
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
                coordArrayX = np.append(coordArrayX,(PixCoordX))
                coordArrayY = np.append(coordArrayY,(PixCoordY))
                radiusArray = np.append(radiusArray,(PixRadius))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))
#                realCoordZ = PixCoordY*(tempObjDiaReal/tempObjDiaPx)
                try:
                    if framenum % updateRatePos == 0:
                        realCoordZ = int((PixCoordY*(tempObjDiaReal/tempObjDiaPx))*20)
                        printCoordsGui(self, PixCoordX, 0,PixCoordY)
                    else:
                        None
                except:
                    None
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

            writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray,framenum,fpsVar,self.PIDIncl)
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


    def objectDetectionOther(self,selected,PIDIncl,reticleIncl):

        if selected == 3:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 1024
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
            createConfigPID(posZ,P,I,D)
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
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (150, 120, 120) #default: 120, 90, 90
                print("Set HSV default values ")
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)

            # if a video path was not supplied, grab the reference
            # to the webcam
            if not args.get("video", False):
                print("Starting video stream...")
    #            vs = VideoStream(src=0).start()
                #-threaded videostream
                vs0 = WebcamVideoStream(src=0).start()
                vs1 = WebcamVideoStream(src=1).start()
            # otherwise, grab a reference to the video file
            else:
                print(" No video stream possible")
            # allow the camera or video file to warm up
            time.sleep(0.2)
            deltaWidth0 = 100
            deltaHeight0 = 100
            deltaWidth1 = deltaWidth0
            deltaHeight1 = deltaHeight0
#                height0, width0, channels = frameCropped0.
#                height1, width1, channels = frame1.shape
# =============================================================================
#            Camera parameters
# =============================================================================
            Cmosx = 5.440 #CMOS width in mm
            Cmosy = 3.072 #CMOS height in mm
            PixCalib = 640 #calibration image  width
            PiyCalib = 360 # calibration image height
            fx0 = 507.41 #focal length x px
            fy0 = 505.94 #focal length y px
            Fx0 = fx0*(Cmosx/PixCalib) #focal length x in mm
            Fy0 = fy0*(Cmosy/PiyCalib) #focal length y in mm
            fx1 = 520.28 #focal length x px
            fy1 = 518.81 #focal length y px
            Fx1 = fx1*(Cmosx/PixCalib) #focal length x in mm
            Fy1 = fx1*(Cmosx/PixCalib) #focal length y in mm
            width1 = 1024
            width0 = width1
            height1 = 576
            height0 = height1
            halfCamWidth0 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight0 = height0/2 #y-offset for centering image coordinate system
            halfCamWidth1 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight1 = height1/2 #y-offset for centering image coordinate system
            halfCamWidth0 = int(halfCamWidth0)
            halfCamHeight0 = int(halfCamHeight0)
            halfCamWidth1 = int(halfCamWidth1)
            halfCamHeight1 = int(halfCamHeight1)
            tracker = None
            writer = None
            confdef = 0.2
            framenum = 0
            fps = FPSOutput().start()

# =============================================================================
# #            Array Initialization
# =============================================================================
            vTrans0 = np.array([])
            vTrans1 = np.array([])
            coordArrayX0 = np.array([])
            coordArrayY0 = np.array([])
            radiusArray0 = np.array([])
            coordArrayX1 = np.array([])
            coordArrayY1 = np.array([])
            radiusArray1 = np.array([])
            xArrayTri = np.array([])
            yArrayTri = np.array([])
            zArrayTri = np.array([])
            timeArray = np.array([])
            tempFrames = np.array([])
            tempPartDiaPixels = np.array([])
            pidOutputArray = np.array([])
            PixCoordX0 = 0
            PixCoordX1 = 0
            PixCoordY0 = 0
            PixCoordY1 = 0

            # keep looping
            while True:
                # grab the current frame
                frame0 = vs0.read()
                frame1 = vs1.read()
                # handle the frame from VideoCapture or VideoStream
                frame0 = frame0[1] if args.get("video", False) else frame0
                frame1 = frame1[1] if args.get("video", False) else frame1
                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if frame0 is None:
                    print("No video preview of camera 0 possible")
                    break
                if frame1 is None:
                    print("No video preview of camera 1 possible")
                    break
                # resize the frame, blur it, and convert it to the HSV
                # color space
#                frame = imutils.resize(frame, width=frameWidth)
                frameCropped0 = imutils.resize(frame0, width=frameWidth)
                frameCropped1 = imutils.resize(frame1, width= frameWidth)
#                frameCropped0 = imutils.resize(frame0, width=frameWidth)
#                frameCropped1 = imutils.resize(frame1, width= frameWidth)
#                halfCamWidth = int(cam_width/2)
#                halfCamHeight = int(cam_height/2)
#                try:
#                    frameCropped0[0:576,0:(450+(int(PixCoordX0))),:] = 255 #left side bar
#                    frameCropped0[188:400,(620+int(PixCoordX0)):1024,:] = 255 #right side bar
#                    frameCropped0[0:220-int(PixCoordY0),0:1024,:] = 255 #upper cross bar
#                    frameCropped0[326-int(PixCoordY0):576,0:1024,:] = 255 #lower cross bar
#                except:
#                    frameCropped0[0:576,0:450,:] = 255 #left side bar
#                    frameCropped0[188:400,620:1024,:] = 255 #right side bar
#                    frameCropped0[0:220,0:1024,:] = 255 #upper cross bar
#                    frameCropped0[326:576,0:1024,:] = 255 #lower cross bar


#                try:
#                    frameCropped1[0:576,0:(450+(int(PixCoordX1))),:] = 255 #left side bar
#                    frameCropped1[188:400,(620+int(PixCoordX1)):1024,:] = 255 #right side bar
#                    frameCropped1[0:220-int(PixCoordY1),0:1024,:] = 255 #upper cross bar
#                    frameCropped1[326-int(PixCoordY1):576,0:1024,:] = 255 #lower cross bar
#                except:
#                    frameCropped1[0:576,0:450,:] = 255 #left side bar
#                    frameCropped1[188:400,620:1024,:] = 255 #right side bar
#                    frameCropped1[0:220,0:1024,:] = 255 #upper cross bar
#                    frameCropped1[326:576,0:1024,:] = 255 #lower cross bar

                frameCropped10 = frameCropped1[halfCamHeight1-int(deltaHeight1):halfCamHeight1+int(deltaHeight1),halfCamWidth1-int(deltaWidth1):halfCamWidth1+int(deltaWidth1),:]
                frameCropped00 = frameCropped0[halfCamHeight0-int(deltaHeight0):halfCamHeight0+int(deltaHeight0),halfCamWidth0-int(deltaWidth0):halfCamWidth0+int(deltaWidth0),:]
                print(len(frameCropped10))
#                crop_img = frameCropped[xCropped-w1:xCropped+w2, yCropped-h1:yCropped+h2]
#                heightCropped, widthCropped, channelsCropped = crop_img.shape
#                deltaXCropped = widthCropped/2 #x-offset for centering image coordinate system
#                deltaYCropped = heightCropped/2 #y-offset for centering image coordinate system
#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
                blurred0 = cv2.GaussianBlur(frameCropped00, (11, 11), 0)
                blurred1 = cv2.GaussianBlur(frameCropped10, (11, 11), 0)
#                hsv0 = cv2.cvtColor(blurred0, cv2.COLOR_BGR2HSV)
#                hsv1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

    #            grayscale = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    #
                # construct a mask for the color "black", then perform
                # a series of dilations and erosions to remove any small
                # blobs left in the mask
            #    mask = cv2.inRange(hsv, greenLower, greenUpper)
            #    mask = cv2.erode(mask, None, iterations=2)
            #    mask = cv2.dilate(mask, None, iterations=2)

                # mask for black color
                mask10 = cv2.inRange(blurred0, blackLower, blackUpper)
                mask11 = cv2.inRange(blurred1, blackLower, blackUpper)
                mask20 = cv2.erode(mask10, None, iterations=2)
                mask21 = cv2.erode(mask11, None, iterations=2)

                #Apply median filter
                mask30 = cv2.dilate(mask20, None, iterations=1)
                mask31 = cv2.dilate(mask21, None, iterations=1)
                mask40 = cv2.medianBlur(mask30,5)
                mask41 = cv2.medianBlur(mask31,5)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts0 = cv2.findContours(mask30.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
#                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
#                    cv2.CHAIN_APPROX_SIMPLE)
                cnts0 = imutils.grab_contours(cnts0)
#                cnts1 = imutils.grab_contours(cnts1)
                center0 = None
                center1 = None

    #            stopTimer()
                print("Contour length: ",len(cnts0))
                # only proceed if at least one contour was found
                if (len(cnts0) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c0 = max(cnts0, key=cv2.contourArea)
                    ((x0, y0), radius0) = cv2.minEnclosingCircle(c0)
                    print("X0",x0)
                    M0 = cv2.moments(c0)
                    if (int(M0["m00"]) != 0) and (radius0 < 150 and radius0 > 20):
                        center0 = (int(M0["m10"] / M0["m00"]), int(M0["m01"] / M0["m00"]))
#                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
#                        print("Center0[0]",center0[0])
#                        print("Center0[1]",center0[1])
                        PixCoordX0 = (center0[0]-deltaWidth0)
                        PixCoordY0 = (center0[1]-deltaHeight0)*(-1)
                        radius0 = radius0
                        pixDiameter0 = 2*radius0
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center0 = np.nan
                        PixCoordX0 = np.nan
                        PixCoordY0 = np.nan
                        radius0 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius0 > 20 and radius0 < 150):
#                    if (radius0 > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped00, (int(center0[0]), int(center0[1])), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius0))
                        PixRadius0 = radius0
#                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#                        PixCoordX0 = np.nan
#                        PixCoordY0 = np.nan
#                        PixCoordX1 = np.nan
#                        PixCoordY1 = np.nan
                        PixRadius0 = np.nan
#                        PixRadius1 = np.nan
                else:
                    PixCoordX0 = np.nan
                    PixCoordY0 = np.nan
                    radius0 = np.nan
                    PixRadius0 = radius0
#                    PixCoordX1 = np.nan
#                    PixCoordY1 = np.nan
#                    radius1 = np.nan
#                    PixRadius1 = radius
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")



                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                if (len(cnts1) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c1 = max(cnts1, key=cv2.contourArea)
                    ((x1, y1), radius1) = cv2.minEnclosingCircle(c1)

                    M1 = cv2.moments(c1)
                    if (int(M1["m00"]) != 0) and (radius1 < 150 and radius1 > 20):
                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
#                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
#                        print("Center0[0]",center1[0])
#                        print("Center0[1]",center1[1])
                        PixCoordX1 = (center1[0]-deltaWidth1)
#                        PixCoordX1 = (center1[0]-deltaHeight0)
                        PixCoordY1 = (center1[1]-deltaHeight1)*(-1)
#                        PixCoordY1 = (center1[1]-halfCamHeight1)*(-1)
                        radius1 = radius1
                        pixDiameter1 = 2*radius1
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center1 = np.nan
#                        center1 = np.nan
                        PixCoordX1 = np.nan
#                        PixCoordX1 = np.nan
                        PixCoordY1 = np.nan
#                        PicCoordY1 = np.nan
                        radius1 = np.nan
#                        radius1 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius1 > 20 and radius1 < 150):
#                    if (radius > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
                        cv2.circle(frameCropped10, (center1[0], center1[1]), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius1))
#                        PixRadius0 = radius0
                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#                        PixCoordX0 = np.nan
#                        PixCoordY0 = np.nan
#                        PixCoordX1 = np.nan
#                        PixCoordY1 = np.nan
#                        PixRadius0 = np.nan
                        PixRadius1 = np.nan
                else:
#                    PixCoordX0 = np.nan
#                    PixCoordY0 = np.nan
#                    radius0 = np.nan
#                    PixRadius0 = radius
                    PixCoordX1 = np.nan
                    PixCoordY1 = np.nan
                    radius1 = np.nan
                    PixRadius1 = radius1
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")
                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
#                cv2.imshow('HSV', mask1)
#                cv2.imshow('Erode', mask2)
#                cv2.imshow('Dilate', mask3)
#                cv2.imshow("Median Filter", mask4)
                if self.reticleIncl == 1:
#                    height, width, channels = frame.shape
#                    frameCropped0 = frameCropped0.copy()
#                    frameCropped1 = frameCropped1.copy()
                    cv2.circle(frameCropped00, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped00, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped00, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.circle(frameCropped10, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped10, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped10, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2

                #geometric properties
                psi0 = 45*(2*math.pi/360)
                psi1 = 45*(2*math.pi/360)
                phi0 = 15.25*(2*math.pi/360)
                phi1 = 8.25*(2*math.pi/360)
                theta0 = 17*(2*math.pi/360)
                theta1 = 17*(2*math.pi/360)
                #transformation from local to global coordinate system
#                PixCoordX0 = PixCoordX0*(math.cos(psi0))*(math.cos(theta0))
#                PixCoordX1 = PixCoordX1*(math.cos(psi1))*(math.cos(theta1))
#                PixCoordZ0 = (PixCoordY0*(math.cos(phi0)))*(math.cos(psi0))
#                PixCoordZ1 = (PixCoordY1*(math.cos(phi1)))*(math.cos(psi1))
#                PixCoordY0 = (PixCoordX0*(math.sin(phi0)))*(math.cos(psi0))
#                PixCoordY1 = (PixCoordX1*(math.sin(phi1)))*(math.cos(psi1))
# =============================================================================
# XYZ Coordinates with Triangulation
# =============================================================================
#                #camera 0
#                PixCoordX0Cam = PixCoordX0
##                PixCoordY0Cam = PixCoordX0
#                PixCoordZ0Cam = PixCoordY0
#                PixCoordX0 = PixCoordX0*((math.cos(psi0))*(math.cos(theta0))) #x Coordinate absolute coordinate system
#                PixCoordY0 = (PixCoordZ0Cam*(math.cos(psi0))*math.sin(theta0)) #y Coordinate absolute coordinate system
#                PixCoordZ0 = (PixCoordZ0Cam*(math.cos(psi0)))+(math.sin(phi0)) #z Coordinate absolute coordinate system
#
#
#                #camera 1
#                PixCoordX1Cam = PixCoordX1
##                PixCoordY1Cam = PixCoordX1
#                PixCoordZ1Cam = PixCoordY1
#                PixCoordX1 = PixCoordX1*((math.cos(psi1))*(math.cos(theta1))) #x Coordinate absolute coordinate system
#                PixCoordY1 = (PixCoordZ1Cam*(math.cos(psi1))*math.sin(theta1)) #y Coordinate absolute coordinate system
#                PixCoordZ1 = (PixCoordZ1Cam*(math.cos(psi1)))+(math.sin(phi1)) #z Coordinate absolute coordinate system
#
#
#                xTri = math.sqrt(PixCoordX0**2+PixCoordX1**2)
##                yTri = math.sqrt(PixCoordY0**2+PixCoordY1**2)
##                zTri = math.sqrt(PixCoordZ0**2+PixCoordZ1**2)
#                zTri =PixCoordZ0
#                yTri =PixCoordY1
#                yTri0 =PixCoordY0
#                yTri1 =PixCoordY1
#
#                print("XTri-Cam0",PixCoordX0)
#                print("XTri-Cam1",PixCoordX1)
#                print("YTri-Cam0",yTri0)
#                print("YTri-Cam1",yTri1)

# =============================================================================
# XYZ Coordinates using rot.matrix and angle of attack phi to generate 90 degrees rotated cameras
# =============================================================================
                cphi0 = np.cos(phi0)
                sphi0 = np.sin(phi0)
                cphi1 = np.cos(phi0)
                sphi1 = np.sin(phi0)

                #rotation around phi axis
                Rphi0 = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
                Rphi1 = np.array(((cphi1, sphi1),(-sphi1, cphi1)))
                vphi0 = np.array((PixCoordX0, PixCoordY0))
                vphi1 = np.array((PixCoordX1, PixCoordY1))
                vphi0 = vphi0.dot(Rphi0) #
                vphi1 = vphi1.dot(Rphi1)
                print("V0",vphi0)
                print("V1",vphi1)

                Rglobal = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
                Rglobal3D = np.array(((1, 0, 0), (0, np.cos(psi0), -np.sin(psi0)), (0, np.sin(psi0), np.cos(psi0))))

                vpsi0 = np.array


                xTriLocal = (vphi0[0]+vphi1[0])/2
                yTriLocal = vphi0[1]
                zTriLocal = vphi1[1]

                vGlobal = np.array((xTriLocal, yTriLocal, zTriLocal))
                vGlobal = vGlobal.dot(Rglobal3D)


                xTri = vGlobal[0]
                zTri = vGlobal[1]
                yTri = vGlobal[2]

                print('X',xTri)
                print('Y',yTri)
                print('Z',zTri)

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Cropped Frame 0", frameCropped00)
                cv2.imshow("Cropped Frame 1", frameCropped10)
                cv2.imshow("Mask30", mask30)
                cv2.imshow("Mask31", mask31)
                key = cv2.waitKey(1) & 0xFF

                if self.PIDIncl == 1:
                    pid = readConfigPID()
                    pid.SetPoint = float(spinBoxValZ)
                    pid.setSampleTime(0.001)
                    pid.setKp(1.1) #default: 3.1
                    pid.setKi(200) #default: 89.7
                    pid.setKd(0.025) #default: 0.025
#                    pid.update(PixCoordY)
                    pid.update(zTri)
                    pidOutputVal = float(pid.output)
                    print("PID output",pid.output)

                else:
                    pidOutputVal = np.nan

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

                #open-loop control adjusted via the user interface
                objZPosZ = setObjPosZ(self, spinBoxValZ)
                print("Obj Z position value: ",objZPosZ)
                if (objZPosZ >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(objZPosZ)]
                    byte2 = dataByte2[int(objZPosZ)]
                    byte3 = dataByte3[int(objZPosZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)
                elif (objZPosZ < 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(720 + objZPosZ)]
                    byte2 = dataByte2[int(720 + objZPosZ)]
                    byte3 = dataByte3[int(720 + objZPosZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)


#                #closed-loop control adjusted via the user interface
                if (pidOutputVal <= 0 and self.PIDIncl == 1):
                    objZPosCLX = (pidOutputVal/20)
                    byte1 = dataByte1[int(719 + objZPosCLX)]
                    byte2 = dataByte2[int(719 + objZPosCLX)]
                    byte3 = dataByte3[int(719 + objZPosCLX)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                elif (pidOutputVal > 0 and self.PIDIncl == 1):
                    objZPosCLZ = (pidOutputVal/20)
                    byte1 = dataByte1[int(objZPosCLZ+objZPosZ)]
                    byte2 = dataByte2[int(objZPosCLZ+objZPosZ)]
                    byte3 = dataByte3[int(objZPosCLZ+objZPosZ)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                try:
                    tempPartDiaPixels = np.append(tempPartDiaPixels,pixDiameter0)
                except:
                    None
                pidOutputArray = np.append(pidOutputArray,pidOutputVal)
                coordArrayX0 = np.append(coordArrayX0,abs(PixCoordX0))
                coordArrayY0 = np.append(coordArrayY0,abs(PixCoordY0))
                radiusArray0 = np.append(radiusArray0,abs(PixRadius0))
                coordArrayX1 = np.append(coordArrayX1,abs(PixCoordX1))
                coordArrayY1 = np.append(coordArrayY1,abs(PixCoordY1))
                radiusArray1 = np.append(radiusArray1,abs(PixRadius1))
                xArrayTri = np.append(xArrayTri,(xTri))
                yArrayTri = np.append(yArrayTri,(yTri))
                zArrayTri = np.append(zArrayTri,(zTri))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))
                printCoordsGui(self, xTri, yTri, zTri)
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

            writePixelPositionPCCam1(timeArray,coordArrayX0,coordArrayY0,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPositionPCCam2(timeArray,coordArrayX1,coordArrayY1,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPosition3D(timeArray,xArrayTri,yArrayTri,zArrayTri,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePIDOutput(timeArray,coordArrayY0,pidOutputArray)

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs0.stop()
                vs1.stop()

            # otherwise, release the camera
            else:
                vs0.release()
                vs1.release()
            # close all windows
            cv2.destroyAllWindows()

#    def _objectDetectionUSBCamera(self,selected,reticleIncl):
#
#        threading.Thread(target=self.objectDetectionUSBCamera).start()

###########################################################
#### USB camera setup for Cropped windows with deep vision
###########################################################

    def objectDetectionCropped(self,selected,PIDIncl,reticleIncl):


        if selected == 4:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 1024
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
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (150, 120, 120) #default: 120, 90, 90
                print("Set HSV default values ")
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)

            # if a video path was not supplied, grab the reference
            # to the webcam
            if not args.get("video", False):
                print("Starting video stream...")
    #            vs = VideoStream(src=0).start()
                #-threaded videostream
                vs0 = WebcamVideoStream(src=0).start()
                vs1 = WebcamVideoStream(src=1).start()
            # otherwise, grab a reference to the video file
            else:
                print(" No video stream possible")
            # allow the camera or video file to warm up
            time.sleep(0.2)
            deltaWidth0 = 100
            deltaHeight0 = 100
            deltaWidth1 = deltaWidth0
            deltaHeight1 = deltaHeight0
#                height0, width0, channels = frameCropped0.
#                height1, width1, channels = frame1.shape
            width1 = 1024
            width0 = width1
            height1 = 576
            height0 = height1

            halfCamWidth0 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight0 = height0/2 #y-offset for centering image coordinate system
            halfCamWidth1 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight1 = height1/2 #y-offset for centering image coordinate system
            halfCamWidth0 = int(halfCamWidth0)
            halfCamHeight0 = int(halfCamHeight0)
            halfCamWidth1 = int(halfCamWidth1)
            halfCamHeight1 = int(halfCamHeight1)
            tracker = None
            writer = None
            confdef = 0.2
            framenum = 0
            fps = FPSOutput().start()
            coordArrayX0 = np.array([])
            coordArrayY0 = np.array([])
            radiusArray0 = np.array([])
            coordArrayX1 = np.array([])
            coordArrayY1 = np.array([])
            radiusArray1 = np.array([])
            timeArray = np.array([])
            tempFrames = np.array([])
            tempPartDiaPixels = np.array([])
            pidOutputArray = np.array([])
            PixCoordX0 = 0
            PixCoordX1 = 0
            PixCoordY0 = 0
            PixCoordY1 = 0

            # keep looping
            while True:
                # grab the current frame
                frame0 = vs0.read()
                frame1 = vs1.read()
                # handle the frame from VideoCapture or VideoStream
                frame0 = frame0[1] if args.get("video", False) else frame0
                frame1 = frame1[1] if args.get("video", False) else frame1
                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if frame0 is None:
                    print("No video preview of camera 0 possible")
                    break
                if frame1 is None:
                    print("No video preview of camera 1 possible")
                    break
                # resize the frame, blur it, and convert it to the HSV
                # color space
#                frame = imutils.resize(frame, width=frameWidth)
                frameCropped0 = imutils.resize(frame0, width=frameWidth)
                frameCropped1 = imutils.resize(frame1, width= frameWidth)
#                frameCropped0 = imutils.resize(frame0, width=frameWidth)
#                frameCropped1 = imutils.resize(frame1, width= frameWidth)
#                halfCamWidth = int(cam_width/2)
#                halfCamHeight = int(cam_height/2)
#                try:
#                    frameCropped0[0:576,0:(450+(int(PixCoordX0))),:] = 255 #left side bar
#                    frameCropped0[188:400,(620+int(PixCoordX0)):1024,:] = 255 #right side bar
#                    frameCropped0[0:220-int(PixCoordY0),0:1024,:] = 255 #upper cross bar
#                    frameCropped0[326-int(PixCoordY0):576,0:1024,:] = 255 #lower cross bar
#                except:
#                    frameCropped0[0:576,0:450,:] = 255 #left side bar
#                    frameCropped0[188:400,620:1024,:] = 255 #right side bar
#                    frameCropped0[0:220,0:1024,:] = 255 #upper cross bar
#                    frameCropped0[326:576,0:1024,:] = 255 #lower cross bar


#                try:
#                    frameCropped1[0:576,0:(450+(int(PixCoordX1))),:] = 255 #left side bar
#                    frameCropped1[188:400,(620+int(PixCoordX1)):1024,:] = 255 #right side bar
#                    frameCropped1[0:220-int(PixCoordY1),0:1024,:] = 255 #upper cross bar
#                    frameCropped1[326-int(PixCoordY1):576,0:1024,:] = 255 #lower cross bar
#                except:
#                    frameCropped1[0:576,0:450,:] = 255 #left side bar
#                    frameCropped1[188:400,620:1024,:] = 255 #right side bar
#                    frameCropped1[0:220,0:1024,:] = 255 #upper cross bar
#                    frameCropped1[326:576,0:1024,:] = 255 #lower cross bar

                frameCropped10 = frameCropped1[halfCamHeight1-int(deltaHeight1):halfCamHeight1+int(deltaHeight1),halfCamWidth1-int(deltaWidth1):halfCamWidth1+int(deltaWidth1),:]
                frameCropped00 = frameCropped0[halfCamHeight0-int(deltaHeight0):halfCamHeight0+int(deltaHeight0),halfCamWidth0-int(deltaWidth0):halfCamWidth0+int(deltaWidth0),:]
                print(len(frameCropped10))
#                crop_img = frameCropped[xCropped-w1:xCropped+w2, yCropped-h1:yCropped+h2]
#                heightCropped, widthCropped, channelsCropped = crop_img.shape
#                deltaXCropped = widthCropped/2 #x-offset for centering image coordinate system
#                deltaYCropped = heightCropped/2 #y-offset for centering image coordinate system
#                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#                edged = cv2.Canny(gray,50,100)
                blurred0 = cv2.GaussianBlur(frameCropped00, (11, 11), 0)
                blurred1 = cv2.GaussianBlur(frameCropped10, (11, 11), 0)
#                hsv0 = cv2.cvtColor(blurred0, cv2.COLOR_BGR2HSV)
#                hsv1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

    #            grayscale = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    #
                # construct a mask for the color "black", then perform
                # a series of dilations and erosions to remove any small
                # blobs left in the mask
            #    mask = cv2.inRange(hsv, greenLower, greenUpper)
            #    mask = cv2.erode(mask, None, iterations=2)
            #    mask = cv2.dilate(mask, None, iterations=2)

                # mask for black color
                mask10 = cv2.inRange(blurred0, blackLower, blackUpper)
                mask11 = cv2.inRange(blurred1, blackLower, blackUpper)
                mask20 = cv2.erode(mask10, None, iterations=2)
                mask21 = cv2.erode(mask11, None, iterations=2)

                #Apply median filter
                mask30 = cv2.dilate(mask20, None, iterations=1)
                mask31 = cv2.dilate(mask21, None, iterations=1)
                mask40 = cv2.medianBlur(mask30,5)
                mask41 = cv2.medianBlur(mask31,5)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts0 = cv2.findContours(mask30.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
#                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
#                    cv2.CHAIN_APPROX_SIMPLE)
                cnts0 = imutils.grab_contours(cnts0)
#                cnts1 = imutils.grab_contours(cnts1)
                center0 = None
                center1 = None

    #            stopTimer()
                print("Contour length: ",len(cnts0))
                # only proceed if at least one contour was found
                if (len(cnts0) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c0 = max(cnts0, key=cv2.contourArea)
                    ((x0, y0), radius0) = cv2.minEnclosingCircle(c0)
                    print("X0",x0)
                    M0 = cv2.moments(c0)
                    if (int(M0["m00"]) != 0) and (radius0 < 150 and radius0 > 20):
                        center0 = (int(M0["m10"] / M0["m00"]), int(M0["m01"] / M0["m00"]))
#                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
                        print("Center0[0]",center0[0])
                        print("Center0[1]",center0[1])
                        PixCoordX0 = (center0[0]-deltaWidth0)
                        PixCoordY0 = (center0[1]-deltaHeight0)*(-1)
                        radius0 = radius0
                        pixDiameter0 = 2*radius0
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center0 = np.nan
                        PixCoordX0 = np.nan
                        PixCoordY0 = np.nan
                        radius0 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius0 > 20 and radius0 < 150):
#                    if (radius0 > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped00, (int(center0[0]), int(center0[1])), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius0))
                        PixRadius0 = radius0
#                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#                        PixCoordX0 = np.nan
#                        PixCoordY0 = np.nan
#                        PixCoordX1 = np.nan
#                        PixCoordY1 = np.nan
                        PixRadius0 = np.nan
#                        PixRadius1 = np.nan
                else:
                    PixCoordX0 = np.nan
                    PixCoordY0 = np.nan
                    radius0 = np.nan
                    PixRadius0 = radius0
#                    PixCoordX1 = np.nan
#                    PixCoordY1 = np.nan
#                    radius1 = np.nan
#                    PixRadius1 = radius
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")



                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                if (len(cnts1) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c1 = max(cnts1, key=cv2.contourArea)
                    ((x1, y1), radius1) = cv2.minEnclosingCircle(c1)

                    M1 = cv2.moments(c1)
                    if (int(M1["m00"]) != 0) and (radius1 < 150 and radius1 > 20):
                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
#                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
                        print("Center0[0]",center1[0])
                        print("Center0[1]",center1[1])
                        PixCoordX1 = (center1[0]-deltaWidth1)
#                        PixCoordX1 = (center1[0]-deltaHeight0)
                        PixCoordY1 = (center1[1]-deltaHeight1)*(-1)
#                        PixCoordY1 = (center1[1]-halfCamHeight1)*(-1)
                        radius1 = radius1
                        pixDiameter1 = 2*radius1
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center1 = np.nan
#                        center1 = np.nan
                        PixCoordX1 = np.nan
#                        PixCoordX1 = np.nan
                        PixCoordY1 = np.nan
#                        PicCoordY1 = np.nan
                        radius1 = np.nan
#                        radius1 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius1 > 20 and radius1 < 150):
#                    if (radius > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
#                        cv2.circle(frameCropped10, (int(x1), int(y1)), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)
#                        cv2.circle(frameCropped00, (int(x0), int(y0)), int(radius0), (15, 186, 2), 10)
                        cv2.circle(frameCropped10, (center1[0], center1[1]), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius1))
#                        PixRadius0 = radius0
                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#                        PixCoordX0 = np.nan
#                        PixCoordY0 = np.nan
#                        PixCoordX1 = np.nan
#                        PixCoordY1 = np.nan
#                        PixRadius0 = np.nan
                        PixRadius1 = np.nan
                else:
#                    PixCoordX0 = np.nan
#                    PixCoordY0 = np.nan
#                    radius0 = np.nan
#                    PixRadius0 = radius
                    PixCoordX1 = np.nan
                    PixCoordY1 = np.nan
                    radius1 = np.nan
                    PixRadius1 = radius1
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")
                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
#                cv2.imshow('HSV', mask1)
#                cv2.imshow('Erode', mask2)
#                cv2.imshow('Dilate', mask3)
#                cv2.imshow("Median Filter", mask4)
                if self.reticleIncl == 1:
#                    height, width, channels = frame.shape
#                    frameCropped0 = frameCropped0.copy()
#                    frameCropped1 = frameCropped1.copy()
                    cv2.circle(frameCropped00, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped00, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped00, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.circle(frameCropped10, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped10, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped10, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Cropped Frame 0", frameCropped00)
                cv2.imshow("Cropped Frame 1", frameCropped10)
                cv2.imshow("Mask30", mask30)
                cv2.imshow("Mask31", mask31)
                key = cv2.waitKey(1) & 0xFF

                if self.PIDIncl == 1:
                    pid = readConfigPID()
                    pid.SetPoint = float(spinBoxVal)
                    pid.setSampleTime(0.001)
                    pid.setKp(1.1) #default: 3.1
                    pid.setKi(100) #default: 89.7
                    pid.setKd(0.025) #default: 0.025
#                    pid.update(PixCoordY)
                    pid.update(PixCoordY)
                    pidOutputVal = float(pid.output)
                    print("PID output",pid.output)

                else:
                    pidOutputVal = np.nan

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

                #open-loop control adjusted via the user interface
                objZPos = setObjPos(self, spinBoxVal)
                print("Obj Z position value: ",objZPos)
                if (objZPos >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(objZPos)]
                    byte2 = dataByte2[int(objZPos)]
                    byte3 = dataByte3[int(objZPos)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)
                elif (objZPos >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(720 + objZPos)]
                    byte2 = dataByte2[int(720 + objZPos)]
                    byte3 = dataByte3[int(720 + objZPos)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)


#                #open-loop control adjusted via the user interface
                if (pidOutputVal <= 0 and self.PIDIncl == 1):
                    objZPosCL = (pidOutputVal/20)
                    byte1 = dataByte1[int(719 + objZPosCL)]
                    byte2 = dataByte2[int(719 + objZPosCL)]
                    byte3 = dataByte3[int(719 + objZPosCL)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                elif (pidOutputVal > 0 and self.PIDIncl == 1):
                    objZPosCL = (pidOutputVal/20)
                    byte1 = dataByte1[int(objZPosCL)]
                    byte2 = dataByte2[int(objZPosCL)]
                    byte3 = dataByte3[int(objZPosCL)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                try:
                    tempPartDiaPixels = np.append(tempPartDiaPixels,pixDiameter0)
                except:
                    None
                pidOutputArray = np.append(pidOutputArray,pidOutputVal)
                coordArrayX0 = np.append(coordArrayX0,abs(PixCoordX0))
                coordArrayY0 = np.append(coordArrayY0,abs(PixCoordY0))
                radiusArray0 = np.append(radiusArray0,abs(PixRadius0))
                coordArrayX1 = np.append(coordArrayX1,abs(PixCoordX1))
                coordArrayY1 = np.append(coordArrayY1,abs(PixCoordY1))
                radiusArray1 = np.append(radiusArray1,abs(PixRadius1))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))
#                printCoordsGui(self, PixCoordX0, PixCoordY0)
                printCoordsGui(self, PixCoordX0, 0,PixCoordY0)
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

            writePixelPositionPCCam1(timeArray,coordArrayX0,coordArrayY0,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPositionPCCam2(timeArray,coordArrayX1,coordArrayY1,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePIDOutput(timeArray,coordArrayY0,pidOutputArray)

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs0.stop()
                vs1.stop()

            # otherwise, release the camera
            else:
                vs0.release()
                vs1.release()
            # close all windows
            cv2.destroyAllWindows()

#    def _objectDetectionUSBCamera(self,selected,reticleIncl):
#
#        threading.Thread(target=self.objectDetectionUSBCamera).start()


# =============================================================================
#### MultiLev
# =============================================================================

    def objectDetectionMultiLev(self,selected,PIDIncl,reticleIncl):

        if selected == 5:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 1024
                print("Default frame width: ", frameWidth)
            try:
                objDia = int(globObjDia)
            except:
                objDia = 2 #in mm
                print("Default object diameter: ", objDia)

            try:
                tempObjDiaReal = setObjDiamm()
            except:
                tempObjDiaReal = 2.1 #mm
            try:
                tempObjDiaPx = setObjDiapx()
            except:
                tempObjDiaPx = 40 #px

            #geometrical initialization
            psi0 = 45*(2*math.pi/360)
            psi1 = 45*(2*math.pi/360)
            phi0 = 15.25*(2*math.pi/360)
            phi1 = 8.25*(2*math.pi/360)
            theta0 = 17*(2*math.pi/360)
            theta1 = 17*(2*math.pi/360)
            cphi0 = np.cos(phi0)
            sphi0 = np.sin(phi0)
            cphi1 = np.cos(phi0)
            sphi1 = np.sin(phi0)

            #rotation around phi axis
            Rphi0 = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
            Rphi1 = np.array(((cphi1, sphi1),(-sphi1, cphi1)))
            Rglobal = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
            Rglobal3D = np.array(((1, 0, 0), (0, np.cos(psi0), -np.sin(psi0)), (0, np.sin(psi0), np.cos(psi0))))

            #update phase function
            def update(x,y,z):

                for i in range (0,anzTransducer):
                    dist=math.sqrt((transPos[0][i]-x)*(transPos[0][i]-x) + (transPos[1][i]-y)*(transPos[1][i]-y) + (transPos[2][i]-z)*(transPos[2][i]-z))
                    phaseFocal=2*pi*(dist % wavelength)/wavelength


            #upper half
                    if transPos[2][i] > 0:
            #vertical twin trap
                        phase[convertTransducerPosition[i]]=phaseFocal+mainshift

            #stabilising torque (rotation around vertical axis)
                        if transPos[0][i] < 0:
                            phase[convertTransducerPosition[i]]=phase[convertTransducerPosition[i]]
                        else:
                            phase[convertTransducerPosition[i]]=phase[convertTransducerPosition[i]]+secondshift

            #lower half
                    else:
                        phase[convertTransducerPosition[i]] = phaseFocal

            #send phase and duty to FPGA

                for i in range(0,anzTransducer):
            #choose the correction vector and calculate the phase in range [0 .. 719]
                    if i < 36:
                        phaseTot = round(((phase[i] + wiresShiftCorrection1[i]*pi) % (2*pi))/(2*pi)*720) % 720
                    else:
                        phaseTot = round(((phase[i] + wiresShiftCorrection2[i-36]*pi) % (2*pi))/(2*pi)*720) % 720

            #calculate the duty cycle in range [0 .. 59]
                    dutyTot = 0

            #generate the two bytes
                    phaseByte = bytes(int(phaseTot  % 256).to_bytes(1,'big'))
                    dutyByte = bytes(int((dutyTot * 4 + math.floor((phaseTot/256)))).to_bytes(1,'big'))
            #send the bytes
                    serialObject.write(dutyByte)
                    serialObject.write(phaseByte)

            def reset():

                res = 255
                serialObject.write(bytes(res.to_bytes(1,'big')))
                serialObject.write(bytes(res.to_bytes(1,'big')))

            #default settings
            pi=3.14159

            mainshift=pi #phaseshift between the two caps
            secondshift=pi/7 #phaseshift between two halves of the upper cap

            wavelength=8.65 #wavelength for 40 kHz with speed of sound = 346 m/s
            anzTransducer=72 #number of transducers

            #transducer positions (x,y,z)
            transPos = np.array([[17.7143, 8.8571, -8.8571, -17.7143, -8.8571, 8.8571, 33.0553, 24.1982, 8.8571, -8.8571, -24.1982, -33.0553, -33.0553, -24.1982, -8.8571, 8.8571, 24.1982, 33.0553, 51.0063, 47.9302, 39.0731, 25.5031, 8.8571, -8.8571, -25.5031, -39.0731, -47.9302, -51.0063, -47.9302, -39.0731, -25.5031, -8.8571, 8.8571, 25.5031, 39.0731, 47.9302, 17.7143, 8.8571, -8.8571, -17.7143, -8.8571, 8.8571, 33.0553, 24.1982, 8.8571, -8.8571, -24.1982, -33.0553, -33.0553, -24.1982, -8.8571, 8.8571, 24.1982, 33.0553, 51.0063, 47.9302, 39.0731, 25.5031, 8.8571, -8.8571, -25.5031, -39.0731, -47.9302, -51.0063, -47.9302, -39.0731, -25.5031, -8.8571, 8.8571, 25.5031, 39.0731, 47.9302], [0, 15.341, 15.341, 2.1694e-15, -15.341, -15.341, 8.8571, 24.1982, 33.0553, 33.0553, 24.1982, 8.8571, -8.8571, -24.1982, -33.0553, -33.0553, -24.1982, -8.8571, 0, 17.4452, 32.7862, 44.1727, 50.2314, 50.2314, 44.1727, 32.7862, 17.4452, 6.2465e-15, -17.4452, -32.7862, -44.1727, -50.2314, -50.2314, -44.1727, -32.7862, -17.4452, 0, 15.341, 15.341, 2.1694e-15, -15.341, -15.341, 8.8571, 24.1982, 33.0553, 33.0553, 24.1982, 8.8571, -8.8571, -24.1982, -33.0553, -33.0553, -24.1982, -8.8571, 0, 17.4452, 32.7862, 44.1727, 50.2314, 50.2314, 44.1727, 32.7862, 17.4452, 6.2465e-15, -17.4452, -32.7862, -44.1727, -50.2314, -50.2314, -44.1727, -32.7862, -17.4452],[-59.4155, -59.4155, -59.4155, -59.4155, -59.4155, -59.4155, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -51.7001, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, -35.2472, 59.4155, 59.4155, 59.4155, 59.4155, 59.4155, 59.4155, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 51.7001, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472, 35.2472]])

            #compensation for the phase shift due to the connections between the PCB and the transducers
            wiresShiftCorrection1 = np.array([1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0])
            wiresShiftCorrection2 = np.array([1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0,0,0,1,1,1])

            #transfer vector from the number of the transducer position to the number of the PCB output
            convertTransducerPosition = np.array([30,31,32,33,34,35,18,19,20,21,22,23,24,25,26,27,28,29,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36])
            phase = np.zeros(anzTransducer)
            reset()
            update(0,0,0)


            #controller initialization
            global targetPosX
            global targetPosY
            global targetPosZ
            Px = 0.5
            Ix = 1.5
            Dx = 0.3
            Py = 0.5
            Iy = 1.5
            Dy = 0.3
            Pz = 0.5
            Iz = 1.5
            Dz = 0.3
            posX = 0
            posY = 0
            posZ = 0
            targetPosX = 0
            targetPosY = 0
            targetPosZ = 0
            initPIDParams3D(posX, posY, posZ, Px, Ix, Dx, Py, Iy, Dy, Pz, Iz, Dz)
            time.sleep(0.1)
            createConfigPID3D(posX, posY, posZ, Px, Ix, Dx, Py, Iy, Dy, Pz, Iz, Dz)
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
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (150, 120, 120) #default: 120, 90, 90
                print("Set HSV default values ")
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)

            # if a video path was not supplied, grab the reference
            # to the webcam
            if not args.get("video", False):
                print("Starting video stream...")
    #            vs = VideoStream(src=0).start()
                #-threaded videostream
                vs0 = WebcamVideoStream(src=0).start()
                vs1 = WebcamVideoStream(src=1).start()
            # otherwise, grab a reference to the video file
            else:
                print(" No video stream possible")
            # allow the camera or video file to warm up
            time.sleep(0.2)
            deltaWidth0 = 100
            deltaHeight0 = 100
            deltaWidth1 = deltaWidth0
            deltaHeight1 = deltaHeight0
#                height0, width0, channels = frameCropped0.
#                height1, width1, channels = frame1.shape
# =============================================================================
#            Camera parameters
# =============================================================================
            Cmosx = 5.440 #CMOS width in mm
            Cmosy = 3.072 #CMOS height in mm
            PixCalib = 640 #calibration image  width
            PiyCalib = 360 # calibration image height
            fx0 = 507.41 #focal length x px
            fy0 = 505.94 #focal length y px
            Fx0 = fx0*(Cmosx/PixCalib) #focal length x in mm
            Fy0 = fy0*(Cmosy/PiyCalib) #focal length y in mm
            fx1 = 520.28 #focal length x px
            fy1 = 518.81 #focal length y px
            Fx1 = fx1*(Cmosx/PixCalib) #focal length x in mm
            Fy1 = fx1*(Cmosx/PixCalib) #focal length y in mm
            width1 = 1024
            width0 = width1
            height1 = 576
            height0 = height1
            halfCamWidth0 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight0 = height0/2 #y-offset for centering image coordinate system
            halfCamWidth1 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight1 = height1/2 #y-offset for centering image coordinate system
            halfCamWidth0 = int(halfCamWidth0)
            halfCamHeight0 = int(halfCamHeight0)
            halfCamWidth1 = int(halfCamWidth1)
            halfCamHeight1 = int(halfCamHeight1)
            tracker = None
            writer = None
            confdef = 0.2
            framenum = 0
            fps = FPSOutput().start()

# =============================================================================
# #            Array Initialization
# =============================================================================
            vTrans0 = np.array([])
            vTrans1 = np.array([])
            coordArrayX0 = np.array([])
            coordArrayY0 = np.array([])
            radiusArray0 = np.array([])
            coordArrayX1 = np.array([])
            coordArrayY1 = np.array([])
            radiusArray1 = np.array([])
            xArrayTri = np.array([])
            yArrayTri = np.array([])
            zArrayTri = np.array([])
            timeArray = np.array([])
            tempFrames = np.array([])
            tempPartDiaPixels = np.array([])
            pidOutputArray = np.array([])
            PixCoordX0 = 0
            PixCoordX1 = 0
            PixCoordY0 = 0
            PixCoordY1 = 0

            # keep looping
            while True:
                # grab the current frame
                frame0 = vs0.read()
                frame1 = vs1.read()
                # handle the frame from VideoCapture or VideoStream
                frame0 = frame0[1] if args.get("video", False) else frame0
                frame1 = frame1[1] if args.get("video", False) else frame1
                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if frame0 is None:
                    print("No video preview of camera 0 possible")
                    break
                if frame1 is None:
                    print("No video preview of camera 1 possible")
                    break
                # resize the frame, blur it, and convert it to the HSV
                # color space
#                frame = imutils.resize(frame, width=frameWidth)
                frameCropped0 = imutils.resize(frame0, width=frameWidth)
                frameCropped1 = imutils.resize(frame1, width= frameWidth)

                frameCropped10 = frameCropped1[halfCamHeight1-int(deltaHeight1):halfCamHeight1+int(deltaHeight1),halfCamWidth1-int(deltaWidth1):halfCamWidth1+int(deltaWidth1),:]
                frameCropped00 = frameCropped0[halfCamHeight0-int(deltaHeight0):halfCamHeight0+int(deltaHeight0),halfCamWidth0-int(deltaWidth0):halfCamWidth0+int(deltaWidth0),:]
                print(len(frameCropped10))

                blurred0 = cv2.GaussianBlur(frameCropped00, (11, 11), 0)
                blurred1 = cv2.GaussianBlur(frameCropped10, (11, 11), 0)

                # mask for black color
                mask10 = cv2.inRange(blurred0, blackLower, blackUpper)
                mask11 = cv2.inRange(blurred1, blackLower, blackUpper)
                mask20 = cv2.erode(mask10, None, iterations=2)
                mask21 = cv2.erode(mask11, None, iterations=2)

                #Apply median filter
                mask30 = cv2.dilate(mask20, None, iterations=1)
                mask31 = cv2.dilate(mask21, None, iterations=1)
                mask40 = cv2.medianBlur(mask30,5)
                mask41 = cv2.medianBlur(mask31,5)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts0 = cv2.findContours(mask30.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
#                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
#                    cv2.CHAIN_APPROX_SIMPLE)
                cnts0 = imutils.grab_contours(cnts0)
#                cnts1 = imutils.grab_contours(cnts1)
                center0 = None
                center1 = None

    #            stopTimer()
                print("Contour length: ",len(cnts0))
                # only proceed if at least one contour was found
                if (len(cnts0) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c0 = max(cnts0, key=cv2.contourArea)
                    ((x0, y0), radius0) = cv2.minEnclosingCircle(c0)
                    print("X0",x0)
                    M0 = cv2.moments(c0)
                    if (int(M0["m00"]) != 0) and (radius0 < 150 and radius0 > 20):
                        center0 = (int(M0["m10"] / M0["m00"]), int(M0["m01"] / M0["m00"]))

                        PixCoordX0 = (center0[0]-deltaWidth0)
                        PixCoordY0 = (center0[1]-deltaHeight0)*(-1)
                        radius0 = radius0
                        pixDiameter0 = 2*radius0
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center0 = np.nan
                        PixCoordX0 = np.nan
                        PixCoordY0 = np.nan
                        radius0 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius0 > 20 and radius0 < 150):
#                    if (radius0 > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frameCropped00, (int(center0[0]), int(center0[1])), int(radius0), (15, 186, 2), 10)
                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)


                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius0))
                        PixRadius0 = radius0

                    except:
                        print("No radius detected")

                        PixRadius0 = np.nan
                else:
                    PixCoordX0 = np.nan
                    PixCoordY0 = np.nan
                    radius0 = np.nan
                    PixRadius0 = radius0

                    print("No object detected ...")

                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                if (len(cnts1) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c1 = max(cnts1, key=cv2.contourArea)
                    ((x1, y1), radius1) = cv2.minEnclosingCircle(c1)

                    M1 = cv2.moments(c1)
                    if (int(M1["m00"]) != 0) and (radius1 < 150 and radius1 > 20):
                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
                        PixCoordX1 = (center1[0]-deltaWidth1)
                        PixCoordY1 = (center1[1]-deltaHeight1)*(-1)
                        radius1 = radius1
                        pixDiameter1 = 2*radius1
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center1 = np.nan
                        PixCoordX1 = np.nan
                        PixCoordY1 = np.nan
                        radius1 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius1 > 20 and radius1 < 150):
#                    if (radius > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points

                        cv2.circle(frameCropped10, (center1[0], center1[1]), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius1))
#                        PixRadius0 = radius0
                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#
                        PixRadius1 = np.nan
                else:

                    PixCoordX1 = np.nan
                    PixCoordY1 = np.nan
                    radius1 = np.nan
                    PixRadius1 = radius1
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")
                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
#                cv2.imshow('HSV', mask1)
#                cv2.imshow('Erode', mask2)
#                cv2.imshow('Dilate', mask3)
#                cv2.imshow("Median Filter", mask4)
                if self.reticleIncl == 1:
#                    height, width, channels = frame.shape
#                    frameCropped0 = frameCropped0.copy()
#                    frameCropped1 = frameCropped1.copy()
                    cv2.circle(frameCropped00, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped00, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped00, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.circle(frameCropped10, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped10, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped10, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2



# =============================================================================
# XYZ Coordinates using rot.matrix and angle of attack phi to generate 90 degrees rotated cameras
# =============================================================================

                vphi0 = np.array((PixCoordX0, PixCoordY0))
                vphi1 = np.array((PixCoordX1, PixCoordY1))
                vphi0 = vphi0.dot(Rphi0) #
                vphi1 = vphi1.dot(Rphi1)
                print("V0",vphi0)
                print("V1",vphi1)

                xTriLocal = (vphi0[0]+vphi1[0])/2
                yTriLocal = vphi0[1]
                zTriLocal = vphi1[1]

                vGlobal = np.array((xTriLocal, yTriLocal, zTriLocal))
                vGlobal = vGlobal.dot(Rglobal3D)

                xTri = vGlobal[0]
                zTri = vGlobal[1]
                yTri = vGlobal[2]

                print('X',xTri)
                print('Y',yTri)
                print('Z',zTri)

                #transformation from px to mm
                xTrimm = xTri*(tempObjDiaReal/tempObjDiaPx)
                yTrimm = yTri*(tempObjDiaReal/tempObjDiaPx)
                zTrimm = zTri*(tempObjDiaReal/tempObjDiaPx)

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Cropped Frame 0", frameCropped00)
                cv2.imshow("Cropped Frame 1", frameCropped10)
                cv2.imshow("Mask30", mask30)
                cv2.imshow("Mask31", mask31)
                key = cv2.waitKey(1) & 0xFF

                if self.PIDIncl == 1:
                    pidX, pidY, pidZ = readConfigPID3D()
                    pidX.SetPoint = float(spinBoxValX)
                    pidX.setSampleTime(0.001)
                    pidX.setKpx(1.1) #default: 3.1
                    pidX.setKix(200) #default: 89.7
                    pidX.setKdx(0.025) #default: 0.025
#                    pid.update(PixCoordY)
                    pidX.updatex(xTri)
                    pidOutputVal = float(pidX.outputx)
                    print("PID output",pidX.outputx)

                else:
                    pidOutputVal = np.nan

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

                #open-loop control adjusted via the user interface
                valueX = setObjPosX(self, spinBoxValX)
                valueY = setObjPosY(self, spinBoxValY)
                valueZ = setObjPosZ(self, spinBoxValZ)

                objZPosCLX = (pidOutputVal/20)

                update(valueX/100, valueY/100, valueZ/100)



#                objZPosX = setObjPosX(self, spinBoxVal)
#                print("Obj Z position value: ",objZPosX)
#                if (objZPosX >= 0 and self.PIDIncl == 0):
#                    byte1 = dataByte1[int(objZPosX)]
#                    byte2 = dataByte2[int(objZPosX)]
#                    byte3 = dataByte3[int(objZPosX)]
#                    values = bytearray([byte1, byte2, byte3])
#                    serialObject.write(values)
#                    print("Serial Values: ",byte1)
#                elif (objZPos < 0 and self.PIDIncl == 0):
#                    byte1 = dataByte1[int(720 + objZPosX)]
#                    byte2 = dataByte2[int(720 + objZPosX)]
#                    byte3 = dataByte3[int(720 + objZPosX)]
#                    values = bytearray([byte1, byte2, byte3])
#                    serialObject.write(values)
#                    print("Serial Values: ",byte1)


#                #closed-loop control adjusted via the user interface
#                if (pidOutputVal <= 0 and self.PIDIncl == 1):
#                    objZPosCLX = (pidOutputVal/20)
#                    byte1 = dataByte1[int(719 + objZPosCLX)]
#                    byte2 = dataByte2[int(719 + objZPosCLX)]
#                    byte3 = dataByte3[int(719 + objZPosCLX)]
#                    values = bytearray([byte1, byte2, byte3])
#                    serialObject.write(values)
#                    print("Serial Values: ",byte1)
#
#                elif (pidOutputVal > 0 and self.PIDIncl == 1):
#                    objZPosCL = (pidOutputVal/20)
#                    byte1 = dataByte1[int(objZPosCLX+objZPosX)]
#                    byte2 = dataByte2[int(objZPosCLX+objZPosX)]
#                    byte3 = dataByte3[int(objZPosCLX+objZPosX)]
#                    values = bytearray([byte1, byte2, byte3])
#                    serialObject.write(values)
#                    print("Serial Values: ",byte1)

                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                try:
                    tempPartDiaPixels = np.append(tempPartDiaPixels,pixDiameter0)
                except:
                    None
                pidOutputArray = np.append(pidOutputArray,pidOutputVal)
                coordArrayX0 = np.append(coordArrayX0,abs(PixCoordX0))
                coordArrayY0 = np.append(coordArrayY0,abs(PixCoordY0))
                radiusArray0 = np.append(radiusArray0,abs(PixRadius0))
                coordArrayX1 = np.append(coordArrayX1,abs(PixCoordX1))
                coordArrayY1 = np.append(coordArrayY1,abs(PixCoordY1))
                radiusArray1 = np.append(radiusArray1,abs(PixRadius1))
                xArrayTri = np.append(xArrayTri,(xTri))
                yArrayTri = np.append(yArrayTri,(yTri))
                zArrayTri = np.append(zArrayTri,(zTri))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))
                printCoordsGui(self, xTrimm, yTrimm, zTrimm)
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

            writePixelPositionPCCam1(timeArray,coordArrayX0,coordArrayY0,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPositionPCCam2(timeArray,coordArrayX1,coordArrayY1,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPosition3D(timeArray,xArrayTri,yArrayTri,zArrayTri,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePIDOutput(timeArray,coordArrayY0,pidOutputArray)

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs0.stop()
                vs1.stop()

            # otherwise, release the camera
            else:
                vs0.release()
                vs1.release()
            # close all windows and close serial connection
            cv2.destroyAllWindows()
            serialObject.close()


class GUI():

    def __init__(self, master,cameraChoice, *args, **kwargs):
#        Frame.__init__(self, master, *args, **kwargs)
        super().__init__(*args, **kwargs)
#         kwargs["style"] = "CustomNotebook"
#         ttk.Notebook.__init__(self, *args, **kwargs)
        self.master = master
#        self.master = tk.Toplevel(self.master)
#        self.frame = Frame(self.master)
        self.master.title("TinyLev")
        self.master.geometry("1100x900+1000+600")
#        self.master.configure(background ='Gray'

#        selected = tk.IntVar()
        self.cameraChoice = cameraChoice
        self.choice = tk.IntVar()
        self.label1 = tk.Label(self.master, text = "Select Camera").grid(row=0, sticky=tk.W, columnspan=3)

        self.master.note = tk.N

        varSerialX = StringVar(self.master) #default value for serial print
        varSerialX.set("0")

        varSerialY = StringVar(self.master) #default value for serial print
        varSerialY.set("0")

        varSerialZ = StringVar(self.master) #default value for serial print
        varSerialZ.set("0")

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

        self.b6 = Button(self.master, text = "Position-Time", command = self.posvsTime)
        self.b6.grid(column= 8, row = 5,sticky = tk.W+tk.E, columnspan =1)

        self.b7 = Button(self.master, text = "Z-Closed-Loop", command = self.newWinZCL)
        self.b7.grid(column= 8, row = 6,sticky = tk.W+tk.E, columnspan =1)

        self.b8 = Button(self.master, text = "Measuring Z-Position", command = self.newWinZOL)
        self.b8.grid(column= 8, row = 7,sticky = tk.W+tk.E, columnspan =1)

        self.b9 = Button(self.master, text = "Show FFT", command = self.showFFTfcn)
        self.b9.grid(column= 8, row = 8,sticky = tk.W+tk.E, columnspan =1)

        self.setPnt = tk.Label(self.master, text="H-Value: ")
        self.setPnt.grid(column=2,row=11,sticky = tk.W+tk.E)

        self.spinBoxH = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueH)
        self.spinBoxH.grid(column=7, row= 11,sticky = tk.W+tk.E)

        self.setPnt = tk.Label(self.master, text="S-Value: ")
        self.setPnt.grid(column=2,row=12,sticky = tk.W+tk.E)

        self.spinBoxS = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueS)
        self.spinBoxS.grid(column=7, row= 12,sticky = tk.W+tk.E)

        self.setPnt = tk.Label(self.master, text="V-Value: ")
        self.setPnt.grid(column=2,row=13,sticky = tk.W+tk.E)

        self.spinBoxV = tk.Spinbox(self.master, from_ =0, to=255, command = self.printValueV)
        self.spinBoxV.grid(column=7, row= 13,sticky = tk.W+tk.E)

        self.b10 = Button(self.master, text="Set Values ", command = self.setAllValuesHSV)
        self.b10.grid(column=8,row=12,sticky = tk.W)

        self.b11 = Button(self.master, text="Default ", command = self.setHSVDef)
        self.b11.grid(column=8,row=13,sticky = tk.W)

        self.b12 = Button(self.master, text="Exit ", command = closeAll)
        self.b12.grid(column=8,row=9,sticky = tk.W+tk.E,columnspan =1)

        self.varRetBox = IntVar()
        self.RetBox = Checkbutton(self.master, text = "Include Reticle", variable=self.varRetBox,command=self.setReticleIncl)
        self.RetBox.grid(column=1,row=10,sticky = tk.W,columnspan =1)

        self.xCoordLbl = tk.Label(self.master, text="X-Coordinate: --:--")
        self.xCoordLbl.grid(column=2,row=14,sticky = tk.W+tk.E)

        self.yCoordLbl = tk.Label(self.master, text="Y-Coordinate: --:--")
        self.yCoordLbl.grid(column=2,row=15,sticky = tk.W+tk.E)

        self.zCoordLbl = tk.Label(self.master, text="Z-Coordinate: --:--")
        self.zCoordLbl.grid(column=2,row=16,sticky = tk.W+tk.E)

        self.setPntFrame = tk.Label(self.master, text="Define Frame Width: ")
        self.setPntFrame.grid(column=2,row=17,sticky = tk.W+tk.E)

        self.spinBoxWidth = tk.Spinbox(self.master, from_ =1, to=1024, command = self.printFrameWidth)
        self.spinBoxWidth.grid(column=7, row= 17,sticky = tk.W+tk.E)

        self.b13 = Button(self.master, text="Set Width ", command = self.setFrameWidth)
        self.b13.grid(column=8,row=17,sticky = tk.W)

        self.setPntFrame = tk.Label(self.master, text="Define Object Diameter/ mm: ")
        self.setPntFrame.grid(column=2,row=18,sticky = tk.W+tk.E)

        self.spinBoxObjDiamm = tk.Spinbox(self.master, from_ =1, to=1024, command = self.setObjDiamm)
        self.spinBoxObjDiamm.grid(column=7, row= 18,sticky = tk.W+tk.E)

        self.b14 = Button(self.master, text="Set Diameter ", command = self.setObjDiamm)
        self.b14.grid(column=8,row=18,sticky = tk.W)

        self.setPntObjDia = tk.Label(self.master, text="Define Object Diameter/ px: ")
        self.setPntObjDia.grid(column=2,row=19,sticky = tk.W+tk.E)

        self.spinBoxObjDiaPx = tk.Spinbox(self.master, from_ =0.1, to=200, command = self.setObjDiapx)
        self.spinBoxObjDiaPx.grid(column=7, row= 19,sticky = tk.W+tk.E)

        self.b20 = Button(self.master, text="Set Diameter ", command = self.setObjDiapx)
        self.b20.grid(column=8,row=19,sticky = tk.W)

        self.b15 = Button(self.master, text="Plot PID ", command = self.setPIDPlot)
        self.b15.grid(column=7,row=8,sticky = tk.W+tk.E,columnspan =1)


        self.b17 = Button(self.master, text = "Connect FPGA", command = connectFPGACL)
        self.b17.grid(column= 7, row = 4, sticky = tk.W+tk.E)

        self.b18 = Button(self.master, text = "Close FPGA", command = closeFPGACL)
        self.b18.grid(column= 7, row = 5, sticky = tk.W+tk.E)

        self.varPIDBox = IntVar()
        self.PIDBox = Checkbutton(self.master, text = "Include PID     ", variable=self.varPIDBox,command=self.setPIDIncl)
        self.PIDBox.grid(column=1,row=9,sticky = tk.W,columnspan =1)

        self.b19 = Button(self.master, text="Camera Calibration ", command = self.openCamCalib)
        self.b19.grid(column=7,row=9,sticky = tk.W+tk.E,columnspan =1)

        self.b20 = Button(self.master, text="3D-Plot ", command = self.set3DPlot)
        self.b20.grid(column=7,row=10,sticky = tk.W+tk.E,columnspan =1)

        self.setPosFrameX = tk.Label(self.master, text="Define Object Position X: ")
        self.setPosFrameX.grid(column=2,row=20,sticky = tk.W+tk.E)

        self.spinBoxObjPosX = tk.Spinbox(self.master, from_ =-720, to=720,textvariable = varSerialX, command = self.setObjPosTempX)
        self.spinBoxObjPosX.grid(column=7, row= 20,sticky = tk.W+tk.E)

        self.b16 = Button(self.master, text="Set Position ", command = self.setObjPosTempX)
        self.b16.grid(column=8,row=20,sticky = tk.W)

        self.setPosFrameY = tk.Label(self.master, text="Define Object Position Y: ")
        self.setPosFrameY.grid(column=2,row=21,sticky = tk.W+tk.E)

        self.spinBoxObjPosY = tk.Spinbox(self.master, from_ =-720, to=720,textvariable = varSerialY, command = self.setObjPosTempY)
        self.spinBoxObjPosY.grid(column=7, row= 21,sticky = tk.W+tk.E)

        self.b21 = Button(self.master, text="Set Position ", command = self.setObjPosTempY)
        self.b21.grid(column=8,row=21,sticky = tk.W)

        self.setPosFrameZ = tk.Label(self.master, text="Define Object Position Z: ")
        self.setPosFrameZ.grid(column=2,row=22,sticky = tk.W+tk.E)

        self.spinBoxObjPosZ = tk.Spinbox(self.master, from_ =-720, to=720,textvariable = varSerialZ, command = self.setObjPosTempZ)
        self.spinBoxObjPosZ.grid(column=7, row= 22,sticky = tk.W+tk.E)

        self.b22 = Button(self.master, text="Set Position ", command = self.setObjPosTempZ)
        self.b22.grid(column=8,row=22,sticky = tk.W)


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


    def setObjPosTempX(self):
        global spinBoxValX
        spinBoxValX = self.spinBoxObjPosX.get()
        setObjPosX(self, spinBoxValX)


    def setObjPosTempY(self):
        global spinBoxValY
        spinBoxValY = self.spinBoxObjPosY.get()
        setObjPosY(self, spinBoxValY)


    def setObjPosTempZ(self):
        global spinBoxValZ
        spinBoxValZ = self.spinBoxObjPosZ.get()
        setObjPosZ(self, spinBoxValZ)

    def setObjDiamm(self):
        print("Object diameter changed manually to: ",self.spinBoxObjDiamm.get())
        global globObjDiamm
        globObjDiamm = int(self.spinBoxObjDiamm.get())
        return globObjDiamm


    def setObjDiapx(self):
        print("Object diameter changed manually to: ",self.spinBoxObjDiaPx.get())
        global globFrameWidth
        globObjDia = int(self.spinBoxObjDiaPx.get())
        return globObjDia


    def setPIDPlot(self):
        showPIDPlot()


    def set3DPlot(self):
        show3DPlot()


    def openCamCalib(self):
        self.master.withdraw()
        self.newWindow = tk.Toplevel(self.master)
        newBtnCamCalib = camCalib(self.newWindow)
        print("Camera calibration setup opened")


    def printFrameWidth(self):
        print("Frame width: ", self.spinBoxWidth.get())


    def setFrameWidth(self):
        print("Frame width changed manually to: ",self.spinBoxWidth.get())
        global globFrameWidth
        globFrameWidth = int(self.spinBoxWidth.get())
        return globFrameWidth


    def showFFTfcn(self):
        showFFT()


    def posvsTime(self):
        showPosvsTime()


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
            self.reticleIncl = self.varRetBox.get()
        except:
            self.reticleIncl = 0
        try:
            self.PIDIncl = self.varPIDBox.get()
        except:
            self.PIDIncl = 0

        if selected == 0:
            objectDetection.objectDetectionPicamera(self,selected,self.PIDIncl,self.reticleIncl)
        elif selected == 1:
            objectDetection.objectDetectionUSBCamera(self,selected,self.PIDIncl,self.reticleIncl)
        elif selected == 3:
            objectDetection.objectDetectionOther(self,selected,self.PIDIncl,self.reticleIncl)
        elif selected == 2:
            objectDetection.objectDetectionZMeasurement(self,selected,self.PIDIncl,self.reticleIncl)
        elif selected == 4:
            objectDetection.objectDetectionCropped(self,selected,self.PIDIncl,self.reticleIncl)
        elif selected == 5:
            objectDetection.objectDetectionMultiLev(self,selected,self.PIDIncl,self.reticleIncl)

    def setPIDIncl(self):
        self.PIDIncl = self.varPIDBox.get()
        if self.varPIDBox.get() == 1:
            print("PID included ...") #if checkbox is activated, value is equak to 1, otherwise 0
        elif self.varPIDBox.get() == 0:
            print("PID not included ...") #if checkbox is activated, value is equak to 1, otherwise 0

    def setReticleIncl(self):
        print("Reticle value:",self.varRetBox.get()) #if checkbox is activated, value is equak to 1, otherwise 0
        self.reticleIncl = self.varRetBox.get()


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


class camCalib():

    def __init__(self, master,  *args, **kwargs):

        super().__init__(*args, **kwargs)
        self.master = master
        self.frame = Frame(self.master)
        self.master.lift()

#        self.stereo_depth_map(self)
        self.master.title("Camera calibration")
        self.master.geometry('600x400+1000+600')


        self.modeChoice = tk.IntVar()
        self.modeMeasuring = modeMeasuring
        selected = tk.IntVar()

        self.stpmsm = tk.Button(self.master, text = "Camera test", command = self.setStereoCameraTest)
#        self.stpmsm.grid(column= 3, row = 4, sticky = tk.W+tk.E,columnspan = 1)
        self.stpmsm.grid(column=3,row=1,sticky = tk.W+tk.E)

        self.shworb = tk.Button(self.master, text = "Chess cycle", command = self.setChessCycle)
#        self.shworb.grid(column= 3, row = 5, sticky =tk.W+tk.E,columnspan = 1)
        self.shworb.grid(column=3,row=2,sticky = tk.W+tk.E)

        self.bck = tk.Button(self.master, text = "Calibration", command = self.setCalibration)
#        self.bck.grid(column= 3, row = 6, sticky = tk.W+tk.E,columnspan = 1)
        self.bck.grid(column=3,row=3,sticky = tk.W+tk.E)

        self.setBtn = tk.Button(self.master, text = "Tuning", command = self.setTuning)
#        self.setBtn.grid(column=4, row = 9,sticky = tk.W+tk.E, columnspan = 1)
        self.setBtn.grid(column=3,row=4,sticky = tk.W+tk.E)

        self.setBtnRT = tk.Button(self.master, text="Real-Time",command = self.setRealTime)
#        self.setPnt.grid(column=2,row=9,sticky = tk.W+tk.E)
        self.setBtnRT.grid(column=3,row=5,sticky = tk.W+tk.E)

        self.setBtnBack = tk.Button(self.master, text="Back",command = self.setBackMain)
#        self.setPnt.grid(column=2,row=9,sticky = tk.W+tk.E)
        self.setBtnBack.grid(column=4,row=4,sticky = tk.W+tk.E)

        self.setBtnClose = tk.Button(self.master, text="Close",command = self.setCloseAll)
#        self.setPnt.grid(column=2,row=9,sticky = tk.W+tk.E)
        self.setBtnClose.grid(column=4,row=5,sticky = tk.W+tk.E)

        self.varRetBox = IntVar()
        self.RetBox = Checkbutton(self.master, text = "Include Reticle", variable=self.varRetBox,command=self.setReticleIncl)
        self.RetBox.grid(column=2,row=2,sticky = tk.W,columnspan =1)

    def setBackMain(self):
#        window.destroy()
        self.master.withdraw()
        self.master = tk.Toplevel(self.master)
        bckBtn = GUI(self.master,cameraChoice)

    def setReticleIncl(self):
        print("Reticle value:",self.varRetBox.get()) #if checkbox is activated, value is equak to 1, otherwise 0
        self.reticleIncl = self.varRetBox.get()


    def setStereoCameraTest(self):
        try:
            self.reticleIncl = self.varRetBox.get()
        except:
            self.reticleIncl = 0
        stereoCameraTest(self.reticleIncl)
        print("Camera test closed")


    def setChessCycle(self):
        startChessCycle()
        print("Chess cycle closed")


    def setCalibration(self):
        start3dCalibration()
        print("3D calibraton closed")


    def setTuning(self):
        depthMapTuning.startdepthMapTuning(self)
        print("Tuning closed")


    def setRealTime(self):
        liveDepthMap.depthMapStereo(self)
        print("Real time depth map closed")


    def setCloseAll(self):
        print("")


    def updateWindow1():
        window1.update()


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
    window1 = tk.Tk()
    appCalibCamera = camCalib(self, window1)