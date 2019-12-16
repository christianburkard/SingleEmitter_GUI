"""
Created on 12.11.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""

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