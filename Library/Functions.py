"""
Created on 26.09.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
from tkinter import *
from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2
import imutils
import time
import datetime
import csv
import time
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import serial





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




def writeBytes(timeArray,Byte1,Byte2,Byte3):
    time.sleep(0.5)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'Byte 1' : Byte1, 'Byte 2' : Byte2, 'Byte 3' : Byte3}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("H:/08_Data/TinyLev/WritingBytes/WritingBytes_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.5)
    print("Data written")



def clicked():
    print(selected.get())


def writeMeanDiameter(framenum,PixCoordX,PixCoordY,PixDiameter,selectedCam):
    if selectedCam == 0:
    #    with open("H:/03_Software/Python/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
    #    with open("H/home/pi/Desktop/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
            print("Writing data to file ..")
    #        log.write("{0},{1},{2},{3}\n".format( framenum + 1 , coordArrayX, coordArrayY, radiusArray)
            coordArray = np.array([])
            #coordArray = np.concatenate((coordArrayX, coordArrayY))
            data = {'framenumber' : framenum,'X Coord' : PixCoordX, 'Y Coord' : PixCoordY, 'Diameter' : PixDiameter}
            df1 = pd.DataFrame(data=data)
            timeActual = time.strftime("%d%m%y-%H%M%S")
            df1.to_csv("/home/pi/Desktop/TinyLev/Logging/MeanParticleDiameter_" + timeActual + ".csv", index=False)
            #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
            time.sleep(0.5)
            print("Data written")


    if selectedCam == 1:
#    with open("H:/03_Software/Python/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
#    with open("H/home/pi/Desktop/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
        print("Writing data to file ..")
#        log.write("{0},{1},{2},{3}\n".format( framenum + 1 , coordArrayX, coordArrayY, radiusArray)
        coordArray = np.array([])
        #coordArray = np.concatenate((coordArrayX, coordArrayY))
        data = {'framenumber' : framenum,'X Coord' : PixCoordX, 'Y Coord' : PixCoordY, 'Diameter' : PixDiameter}
        df1 = pd.DataFrame(data=data)
        timeActual = time.strftime("%d%m%y-%H%M%S")
        df1.to_csv("./Logging/MeanParticleDiameter_" + timeActual + ".csv", index=False)
        #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
        time.sleep(0.5)
        print("Data written")

def timeNow():
    now = datetime.datetime.now().strftime("%S.%f")
    now = str(now)
#    hrs, mins, secs = [float(x) for x in s.split(':')]
#    return(hrs*3600 + mins*60 +secs)
    return(now)


def timeNow():
    now = datetime.datetime.now().strftime("%S.%f")
    now = str(now)
#    hrs, mins, secs = [float(x) for x in s.split(':')]
#    return(hrs*3600 + mins*60 +secs)
    return(now)

def startTimer():
    global starttimer
    starttimer = time.time()
    return starttimer
    print("Timer Started")


def stopTimer():
    global endtimer
    endtimer = time.time()
    print("Execution time /s: {:.9f} ".format(endtimer - starttimer))
    print("Timer Stopped")

def InitializeLogFilePixels():
    with open("/home/pi/Desktop/TinyLev/Logging/OrbitCoordinatesPixel.csv", "a") as log:
        log.write("{0},{1},{2},{3}\n".format("PixCoordX", "PixCoordY", "Object Radius Pixels", "time(s)"))


def InitializeLogFileMeanDiameterPC():
    with open("./Logging/MeanParticleDiameter.csv", "a") as log:
        log.write("{0},{1},{2},{3}\n".format("Frame number","PixCoordX", "PixCoordY", "Diameter"))

def InitializeLogFileMeanDiameterRasp():
    with open("/home/pi/Desktop/TinyLev/Logging/MeanParticleDiameter.csv", "a") as log:
        log.write("{0},{1},{2},{3}\n".format("Frame number","PixCoordX", "PixCoordY", "Diameter"))



def pixcoordinate():
    print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))


def writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray):
    time.sleep(0.5)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("./Logging/OrbitCoordinatesPixel_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.5)
    print("Data written")


def writePixelPositionRasp(timeArray,coordArrayX,coordArrayY,radiusArray):
    time.sleep(0.5)
    print("Writing data to file ...")
    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayX, 'Y Coord' : coordArrayY, 'Radius' : radiusArray}
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("/home/pi/Desktop/TinyLev/Logging/OrbitCoordinatesPixel_" + timeActual + ".csv",na_rep = np.nan, index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.5)
    print("Data written")


def showRadiusvsFramePC():
    print("Radius Distribution ...")
    file = filedialog.askopenfilename()
    #    file = 'H:/03_Software/Python/TinyLev/Logging/OrbitCoordinatesPixel.csv'
    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)

    # Plot the data
    #    x = np.arange(0, elapsedTime, meanTimeFrame)
    frameinit = data[:framemax,0]
    radiusdistr = data[:framemax,3]
    plt.plot(frameinit,radiusdistr , '-ok', color = 'black')
    plt.title("Radius Distribution")
    plt.axis('equal')
    plt.xlabel("frame number")
    plt.ylabel("Radius of Contour / px")
    plt.show()
    print("Radius Distribution Plotting done")

def showRadiusvsFrameRasp():
    print("Radius Distribution ...")
    file = filedialog.askopenfilename()

    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)

    # Plot the data
    timeinit = data[:framemax, 0]
    timelinenew = data[:framemax, 3] - timeinit
    plt.plot(timelinenew, data[:, 2], '-ok', color = 'black')
    plt.title("Radius Distribution")
    plt.axis('equal')
    plt.xlabel("frame number")
    plt.ylabel("Radius of Contour / px")
    plt.show()
    print("Radius Distribution Plotting done")

#def showZvsTime():
#    print("Z position vs time ...")
#    file = filedialog.askopenfilename()
#
#    with open(file, 'r') as f:
#        reader = csv.reader(f, delimiter=',')
#        # get header from first row
#        headers = next(reader)
#        # get all the rows as a list
#        data = list(reader)
#        # transform data into numpy array
#        data = np.array(data).astype(float)
#        framemax = len(data[:,1])
#
#    dtime = np.zeros([framemax])
#    tottime = np.array([])
#
#    timeinit = data[:framemax+1, 0]
#    timelinenew = (data[:framemax, 3] - timeinit)
#    dtime[0] = 0
#
#    for i in range(0,framemax-1):
#    #    dtime[i] = 0
#        dtime[i+1] = timeinit[i+1] - timeinit[i]
#        tottime = np.append(dtime[i],tottime+dtime[i])
#    # Plot the data
#
#    plt.plot(tottime, data[:-1, 2], '-ok', color = 'black')
#    plt.title("Position Distribution")
#    #plt.axis('equal')
#    plt.xlabel("Time t / s")
#    plt.ylabel("Z position z / px")
#    plt.show()
#    print("Z position vs time plotting done")
#
#
#
#def showXvsTime():
#    print("Z position vs time ...")
#    file = filedialog.askopenfilename()
#
#    with open(file, 'r') as f:
#        reader = csv.reader(f, delimiter=',')
#        # get header from first row
#        headers = next(reader)
#        # get all the rows as a list
#        data = list(reader)
#        # transform data into numpy array
#        data = np.array(data).astype(float)
#        framemax = len(data[:,1])
#
#    dtime = np.zeros([framemax])
#    tottime = np.array([])
#
#    timeinit = data[:framemax+1, 0]
#    timelinenew = (data[:framemax, 3] - timeinit)
#    dtime[0] = 0
#
#    for i in range(0,framemax-1):
#    #    dtime[i] = 0
#        dtime[i+1] = timeinit[i+1] - timeinit[i]
#        tottime = np.append(dtime[i],tottime+dtime[i])
#    # Plot the data
#
#    plt.plot(tottime, data[:-1, 1], '-ok', color = 'black')
#    plt.title("Position Distribution")
#    #plt.axis('equal')
#    plt.xlabel("Time t / s")
#    plt.ylabel("Z position z / px")
#    plt.show()
#    print("Z position vs time plotting done")


def showPosvsTime():
    print("Positions vs time ...")
    file = filedialog.askopenfilename()

    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)
        framemax = len(data[:,1])

    dtime = np.zeros([framemax])
    tottime = np.array([])

    timeinit = data[:framemax+1, 0]
    timelinenew = (data[:framemax, 3] - timeinit)
    dtime[0] = 0

    for i in range(0,framemax-1):
    #    dtime[i] = 0
        dtime[i+1] = timeinit[i+1] - timeinit[i]
        tottime = np.append(dtime[i],tottime+dtime[i])

    zCalib = (data[:-1, 2]-400)*(-1)

    plt.subplot(2, 1, 1)
    plt.plot(tottime, data[:-1, 1], '-ok', color = 'black')
    plt.title('Position vs time')
    plt.ylabel('X-coordinate / px')

    plt.subplot(2, 1, 2)
    plt.plot(tottime, zCalib, '-ok', color = 'black')
    plt.xlabel('Time t / s')
    plt.ylabel('Z-coordinate / px')

#    # Plot the data
#    plt.plot(tottime, data[:-1, 2], '-ok', color = 'black')
#    plt.title("Position Distribution")
#    #plt.axis('equal')
#    plt.xlabel("Time t / s")
#    plt.ylabel("Z position z / px")
#    plt.show()
    print("Z position vs time plotting done")





def showOrbit(filepath):
    print("Orbit Plotting ...")
    file = filepath
#    file = '/home/pi/Desktop/GUI/Logging/OrbitCoordinatesPixel.csv'
    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)

    # Plot the data
    plt.plot(data[:, 1], data[:, 2], '-ok', color = 'black')
    plt.title("Orbital Pixel Positions Open-Loop")
    plt.axis('equal')
    plt.xlabel("X Pixel Position / px")
    plt.ylabel("Z Pixel Position / px")
    plt.show()
    print("Orbit Plotting done")


def showMeanDia(pathWriteDia, fpsApprox):
    print("Diameter vs Frames Plotting ...")
    file = pathWriteDia
    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)

    # Plot the data
    time.sleep(1.0)
    plt.figure(1)
    plt.plot(data[:, 0], data[:, 1], '-ok', color = 'black')
    plt.title("Mean Object Diameter")
#    plt.axis('equal')
    plt.xlabel("Frame number")
    plt.ylabel("Diameter / mm")
    plt.show()
    print("Diameter vs Frames Plotting done")


def PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, elapsedTime):
    print("Particle Dia vs Time ...")

    x = np.arange(0, elapsedTime, meanTimeFrame)
    y = tempPartDiaPixels

    # Plot the data
    time.sleep(1.0)
    plt.figure(2)
    plt.plot(x, y, '-ok', color = 'black')
    plt.title("Particle Dia vs Time")
#    plt.axis('equal')
    plt.xlabel("Time / s")
    plt.ylabel("Object Diameter / px")
    plt.axis(xlim=(0, elapsedTime), ylim=(100, 600))
    plt.show()
    print("Particle Dia vs Time done")


def defLookUpSteps():
    global dataByte1
    global dataByte2
    global dataByte3

    filePath = open('./Data/LookUpTable_0_720_Dec.csv')
#    with open(filePath, 'r') as f:
    reader = csv.reader(filePath, delimiter=';')
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
