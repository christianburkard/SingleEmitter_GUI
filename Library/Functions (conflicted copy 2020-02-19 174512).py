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
from Library import PID
from Library import PIDMultiX
from Library import PIDMultiY
from Library import PIDMultiZ
import os.path
from mpl_toolkits import mplot3d



def initPIDParams(posZ,P,I,D):
    #parameter initialization
    global pid
    global initposZ
    global initP
    global initI
    global initD
    initposZ = posZ
    initP = P
    initI = I
    initD = D


    #Controller initialization
    pid = PID.PID(initP,initI,initD)
    pid.SetPoint = posZ


def initPIDParams3D(posX, posY, posZ, Px, Ix, Dx, Py, Iy, Dy, Pz, Iz, Dz):
    #parameter initialization
    global pidX
    global pidY
    global pidZ
    global initposX
    global initposY
    global initposZ
    global initPx
    global initIx
    global initDx
    global initPy
    global initIy
    global initDy
    global initPz
    global initIz
    global initDz
    initposX = posX
    initposY = posY
    initposZ = posZ
    initPx = Px
    initIx = Ix
    initDx = Dx
    initPy = Py
    initIy = Iy
    initDy = Dy
    initPz = Pz
    initIz = Iz
    initDz = Dz


    #Controller initialization
    pidX = PIDMultiX.PIDX(initPx,initIx,initDx)
    pidY = PIDMultiY.PIDY(initPy,initIy,initDy)
    pidZ = PIDMultiZ.PIDZ(initPz,initIz,initDz)
    pidX.SetPointx = posX
    pidY.SetPointy = posY
    pidZ.SetPointz = posZ
#    pid.sample_time(0.001)


def readConfigPID():
    global zPos
    with open('./Data/pid.conf','r') as f:
        config = f.readline().split(',')
        pid.SetPoint = float(config[0])
        targetPosZ = pid.SetPoint
        pid.setKp (float(config[1]))
        pid.setKi (float(config[2]))
        pid.setKd (float(config[3]))
        print("Config file loaded ...")
        return pid


def readConfigPID3D():

    with open('./Data/pid3D.conf','r') as f:
        config = f.readline().split(',')
        pidX.SetPointx = float(config[0])
        pidY.SetPointy = float(config[1])
        pidZ.SetPointz = float(config[2])

        targetPosX = pidX.SetPointx
        targetPosY = pidY.SetPointy
        targetPosZ = pidZ.SetPointz

        pidX.setKpx (float(config[3]))
        pidX.setKix (float(config[4]))
        pidX.setKdx (float(config[5]))
        pidY.setKpy (float(config[6]))
        pidY.setKiy (float(config[7]))
        pidY.setKdy (float(config[8]))
        pidZ.setKpz (float(config[9]))
        pidZ.setKiz (float(config[10]))
        pidZ.setKdz (float(config[11]))
        print("Config file loaded ...")
        return pidX, pidY, pidZ


def createConfigPID(PosZ,initP,initI,initD):
    if not os.path.isfile('./Data/pid.conf'):
        with open('./Data/pid.conf','w') as f:
            f.write('%s,%s,%s,%s'%(PosZ,initP,initI,initD))
    print('Config file written ...')


def createConfigPID3D(posX, posY, posZ, Px, Ix, Dx, Py, Iy, Dy, Pz, Iz, Dz):
    if not os.path.isfile('./Data/pid3D.conf'):
        with open('./Data/pid3D.conf','w') as f:
            f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s'%(posX, posY, posZ,Px,Ix,Dx,Py,Iy,Dy,Pz,Iz,Dz))
    print('3D Config file written ...')

#function prints out serial bytes on console
def funcOpenLoop(listValue):
    global serialObject
    ser = serialObject
    listValue = int(listValue)
    byte1 = dataByte1[listValue]
    byte2 = dataByte2[listValue]
    byte3 = dataByte3[listValue]
    time.sleep(0.0005)
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
    time.sleep(0.2)
    print("Writing data to file ..")

    data = {'Time' : timeArray,'Byte 1' : Byte1, 'Byte 2' : Byte2, 'Byte 3' : Byte3}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("H:/08_Data/TinyLev/WritingBytes/WritingBytes_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")


def clicked():
    print(selected.get())


def writeMeanDiameter(selectedCam,timeArray,coordArrayY,coordArrayX,radiusArray,framenum,PixDiameter):
    if selectedCam == 0:
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
            time.sleep(0.2)
            print("Data written")


    if selectedCam == 1:
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
        time.sleep(0.2)
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


def writePixelPositionPC(timeArray,coordArrayX,coordArrayY,radiusArray,framenum,fpsVar,PIDIncl):
    time.sleep(0.2)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray, 'Framenumber' : framenum, 'FPS' : fpsVar}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    if PIDIncl == 0:
        df1.to_csv("./Logging/OrbitCoordinates_OpenLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    elif PIDIncl == 1:
        df1.to_csv("./Logging/OrbitCoordinates_ClosedLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")


def writePixelPositionRasp(timeArray,coordArrayX,coordArrayY,radiusArray):
    time.sleep(0.2)
    print("Writing data to file ...")
    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayX, 'Y Coord' : coordArrayY, 'Radius' : radiusArray}
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("/home/pi/Desktop/TinyLev/Logging/OrbitCoordinatesPixel_" + timeActual + ".csv",na_rep = np.nan, index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")


def writePixelPositionPCCam1(timeArray,coordArrayX,coordArrayY,radiusArray,framenum,fpsVar,PIDIncl):
    time.sleep(0.2)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray, 'Framenumber' : framenum, 'FPS' : fpsVar}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    if PIDIncl == 0:
        df1.to_csv("./Logging/OrbitCoordinates_Camera1_OpenLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    elif PIDIncl == 1:
        df1.to_csv("./Logging/OrbitCoordinates_Camera1_ClosedLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")


def writePixelPositionPCCam2(timeArray,coordArrayX,coordArrayY,radiusArray,framenum,fpsVar,PIDIncl):
    time.sleep(0.2)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : coordArrayY, 'Y Coord' : coordArrayX, 'Radius' : radiusArray, 'Framenumber' : framenum, 'FPS' : fpsVar}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    if PIDIncl == 0:
        df1.to_csv("./Logging/OrbitCoordinates_Camera2_OpenLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    elif PIDIncl == 1:
        df1.to_csv("./Logging/OrbitCoordinates_Camera2_ClosedLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")

def writePixelPosition3D(timeArray,xTri,yTri,zTri,radiusArray0,framenum,fpsVar,PIDIncl):
    time.sleep(0.2)
    print("Writing data to file ..")
#    coordArray = np.array([])
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'X Coord' : xTri, 'Y Coord' : yTri, 'Z Coord' : zTri, 'Radius' : radiusArray0, 'Framenumber' : framenum, 'FPS' : fpsVar}
#    data = [timeArray, coordArrayX, coordArrayY, radiusArray]
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    if PIDIncl == 0:
        df1.to_csv("./Logging/OrbitCoordinates_3D_OpenLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    elif PIDIncl == 1:
        df1.to_csv("./Logging/OrbitCoordinates_3D_ClosedLoop_" + timeActual + ".csv", na_rep = np.nan,index=False)
    #np.savetxt('H:/03_Software/Python/IncreaseFPSPicamera/Logging/OrbitCoordinatesPixel.csv', [coordArray], fmt = '%d',delimiter = ',')
    time.sleep(0.2)
    print("Data written")


def writePIDOutput(timeArray,coordArrayY,pidOutputArray):
    time.sleep(0.2)
    print("Writing data to file ...")
    #coordArray = np.concatenate((coordArrayX, coordArrayY))
    data = {'Time' : timeArray,'Z-Coordinate' : coordArrayY,'PID Output' : pidOutputArray}
    df1 = pd.DataFrame(data)
    df1.replace('',np.nan, inplace = True)
    timeActual = time.strftime("%d%m%y-%H%M%S")
    df1.to_csv("./Logging/PIDOutput_" + timeActual + ".csv", na_rep = np.nan,index=False)
    time.sleep(0.2)
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
    fig = plt.figure('Radius vs. Frame Number')
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
    meandRad = sum(2*data[:-1,3])/len(data[:,1])
    xCalib = (data[:-1, 1])*(-1)
    zReal = (data[:-1,3])
    zReal = zReal/meandRad

    xReal = (data[:-1,1])*(-1)
    xReal = xReal/meandRad

    yReal = (data[:-1,2])
    yReal = yReal/meandRad

    fig = plt.figure('Position vs. Time')
    plt.subplot(6, 1, 1)
    plt.plot(tottime, data[:-1, 3], '-ok', color = 'black')
    plt.title('Position vs time')
    plt.ylabel('Z-Coordinate / px')


    plt.subplot(6, 1, 2)
    plt.plot(tottime, zReal, '-ok', color = 'black')
    plt.ylabel('Z-Coordinate / mm')


    plt.subplot(6, 1, 3)
    plt.plot(tottime, xCalib, '-ok', color = 'black')
    plt.xlabel('Time t / s')
    plt.ylabel('X-Coordinate / px')

    plt.subplot(6, 1, 4)
    plt.plot(tottime, xReal, '-ok', color = 'black')
    plt.xlabel('Time t / s')
    plt.ylabel('X-Coordinate / mm')

    if headers[2] == str('Y Coord'):
        plt.subplot(6, 1, 5)
        plt.plot(tottime, data[:-1, 2], '-ok', color = 'black')
        plt.xlabel('Time t / s')
        plt.ylabel('Y-Coordinate / px')

        plt.subplot(6, 1, 6)
        plt.plot(tottime, yReal, '-ok', color = 'black')
        plt.xlabel('Time t / s')
        plt.ylabel('Y-Coordinate / mm')

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
    fig = plt.figure('2D - Orbit')
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
    time.sleep(0.2)
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
    time.sleep(0.2)
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

def showPIDPlot():
    print("Plotting FFT ...")
    file = filedialog.askopenfilename(initialdir = './Logging')

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
    timelinenew = (data[:framemax, 0] - timeinit)
    dtime[0] = 0

    for i in range(0,framemax-1):
    #    dtime[i] = 0
        dtime[i+1] = timeinit[i+1] - timeinit[i]
        tottime = np.append(dtime[i],tottime+dtime[i])

    sigLen = len(data[:,1])
    time = data[:,0]
    zPos = data[:-1,1]
    pidOutput = data[:-1,2]
    fig = plt.figure('PID Output')
    plt.subplot(2, 1, 1)
    plt.plot(tottime, zPos, 'k', color = 'black')
#    plt.title('Position vs time. Sampling frequency: ',freqSamp)
    plt.ylabel('Z-Amplitude / mm')
    locs, labels = plt.xticks()

    plt.subplot(2, 1, 2)
    plt.plot(tottime, pidOutput, 'k', color = 'black')
    plt.xlabel('Time  t / s')
    plt.ylabel('Ouptut')
    locs, labels = plt.xticks()

    print("PID output plotting done")

def show3DPlot():

    print("Plotting 3D curve ...")
    file = filedialog.askopenfilename(initialdir = './Logging')

    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get header from first row
        headers = next(reader)
        # get all the rows as a list
        data = list(reader)
        # transform data into numpy array
        data = np.array(data).astype(float)
        framemax = len(data[:,1])

    ax = plt.figure('3D Trajectory')
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    zline = data[:,3]
    xline = data[:,1]
    yline = data[:,2]
    ax.plot3D(xline, yline, zline, 'gray')
#    ax.axis('equal')
    # Data for three-dimensional scattered points
    zdata = data[:,3]
    xdata = data[:,1]
    ydata = data[:,2]
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
#    ax.axis('equal')
    ax.set_xlabel('X-Coordinate')
    ax.set_ylabel('Y_Coordinate')
    ax.set_zlabel('Z-Coordinate');




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