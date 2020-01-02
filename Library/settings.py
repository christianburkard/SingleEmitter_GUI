"""
Created on 12.11.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
import serial
import time
import numpy as np
import cv2
import sys
import pandas as pd
from tkinter import filedialog
import tkinter as tk
import csv



#Initialization of global variables
def initGlob():
    global PiCorrdX
    global PiCoordY
    global selected
    global listValue

    global selectedCam

    global data
    global dataByte1
    global dataByte2
    global dataByte3

    global serFPGA
    serFPGA = []

    print("Global variables imported ..")


def defLookUpTable():
    global dataByte1
    global dataByte2
    global dataByte3
    print("Choose look up table ...")
    filePath = filedialog.askopenfilename()
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
        print("done ...")
        return dataByte1, dataByte2, dataByte3


def defaultLookUp():
    global dataByte1
    global dataByte2
    global dataByte3
#    print("Choose look up table ...")
#    filePath = filedialog.askopenfilename()
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
        print("done ...")
        return dataByte1, dataByte2, dataByte3


def serialPorts():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
