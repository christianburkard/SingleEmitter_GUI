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
from Library.Functions import *
from Library.settings import *



#def funcMode1(serialObject):
#    ser = serialObject
#    if phaseByte <= 720:
#
#        if phaseByte >= 512:
#            byte2 = byte2 + 64
#            phaseByte = phaseByte - 512
#
#        elif phaseByte >= 256:
#            byte2 = byte2 + 32
#            phaseByte = phaseByte -256
#
#        elif phaseByte >= 128:
#            byte2 = byte2 + 16
#            phaseByte = phaseByte - 128
#
#        if byte2 > 208 and byte1 == 127:
#            byte2 = 128
#            phaseByte = 0
#
#
#    byte1 = phaseByte
#    byte2 = byte2 + addressByte
#    print("Phase Byte 1:",byte1)
#    print("Phase Byte 2:",byte2)
#
#    byte1Array = np.append(byte1Array,byte1)
#    byte2Array = np.append(byte2Array,byte2)
#    byte3Array = np.append(byte3Array,byte3)
#    timeArray = np.append(timeArray,time.time())
#
#    phaseByte = phaseByte + 1
#    time.sleep(0.05)
#
#    values = bytearray([byte1, byte2, dutyByte])
#    ser.write(values)
#
#    if byte2 == 240 and byte1 == 127:
#        writeBytes(timeArray,byte1Array,byte2Array,byte3Array)
#        sys.exit()
#
#
#
#def funcMode2(serialObject):
#    ser = serialObject
#
#
#
##counts from 0 to 719 from the loaded look up table
#def funcMode3(serialObject,dataByte1,dataByte2,dataByte3):
#    ser = serialObject
#    i = 0
#    startTimer()
##    for i in  range(0,stepNum-1):
#    while i < 719:
#        byte1 = dataByte1[i]
#        byte2 = dataByte2[i]
#        byte3 = dataByte3[i]
#
#        print("Phase Byte 1:",byte1)
#        print("Phase Byte 2:",byte2)
#        print("Step number ",i)
#        time.sleep(0.1)
#        values = bytearray([byte1, byte2, byte3])
#        ser.write(values)
#        i = i + 5
#        stopTimer()
#
#        if i == 717:
#            i =0



#function prints out serial bytes on console
def funcOpenLoop(listValue):
    global serialObject
    ser = serialObject
    listValue = int(listValue)
    byte1 = dataByte1[listValue]
    byte2 = dataByte2[listValue]
    byte3 = dataByte3[listValue]
    time.sleep(0.01)
    values = bytearray([byte1, byte2, byte3])
    ser.write(values)


#browsing function for look up table files
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


#writing bytes into a csv file at specific folder
def writeBytes(timeArray,Byte1,Byte2,Byte3):
    time.sleep(0.5)
    print("Writing data to file ..")

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


def writeMeanDiameter(selectedCam,timeArray,coordArrayY,coordArrayX,radiusArray,framenum,PixDiameter):
    if selectedCam == 0:
    #    with open("H:/03_Software/Python/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
    #    with open("H/home/pi/Desktop/GUI/Logging/MeanParticleDiameter.csv", "a") as log:
            print("Writing data to file ..")
    #        log.write("{0},{1},{2},{3}\n".format( framenum + 1 , coordArrayX, coordArrayY, radiusArray)
            coordArray = np.array([])
            #coordArray = np.concatenate((coordArrayX, coordArrayY))
            data = {'Time' : timeArray,'framenumber' : framenum,'X Coord' : PixCoordX, 'Y Coord' : PixCoordY, 'Diameter' : PixDiameter}
            df1 = pd.DataFrame(data)
            df1.replace('',np.nan, inplace = True)
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
        data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray, 'Framenumber' : framenum, 'Pixe Diameter' : PixDiameter}
        df1 = pd.DataFrame(data)
        df1.replace('',np.nan, inplace = True)
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


def writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray,framenum):
    time.sleep(0.5)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray, 'Framenumber' : framenum}
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


def showRadiusvsFrame():
    print("Radius Distribution ...")
    file = filedialog.askopenfilename(initialdir = './Logging')
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
    framemax = len(data[:,0])
    frameinit = np.arange(framemax)
    radiusdistr = data[:framemax,3]
    meanDia = sum(data[:,3])/(framemax)

    timeinit = data[1,0]
    timeArray = (data[:, 0] - data[0,0])

    plt.subplot(2,1,1)
    plt.plot(frameinit,radiusdistr , '-ok', color = 'black')
    plt.title("Radius Distribution")
    #plt.axis('equal')
    plt.legend(['Mean diameter =' + str(meanDia) +' px'],loc='upper right')
    plt.xlabel("frame number")
    plt.ylabel("Radius of Contour / px")


    plt.subplot(2,1,2)
    plt.plot(timeArray,radiusdistr , '-ok', color = 'black')
    plt.title("Radius Distribution")
    #plt.axis('equal')
    plt.xlabel("Time t / s")
    plt.ylabel("Radius of Contour / px")

    print("Radius Distribution Plotting done")
    print("Mean diameter is / px: ", meanDia)


def showRadiusvsFrameRasp():
    print("Radius Distribution ...")
    file = filedialog.askopenfilename(initialdir = './Logging')

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

    zCalib = (data[:-1, 2])*(-1)

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





def showOrbit():
    print("Orbit Plotting ...")
    file = filedialog.askopenfilename()
#    file = filepath
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
    plt.plot(data[:, 2], data[:, 1], '-ok', color = 'black')
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

