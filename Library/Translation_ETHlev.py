# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 09:50:18 2020

@author: roethlisberger
"""

# -*- coding: utf-8 -*-
import serial
import time
from tkinter import *
from scipy.special import *
from numpy.linalg import *
from numpy import *
import math  

UARTspeed=250000
com='com5'



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
        data.write(dutyByte)    
        data.write(phaseByte)     

def reset(): 

    res = 255
    data.write(bytes(res.to_bytes(1,'big')))
    data.write(bytes(res.to_bytes(1,'big')))    
    

data = serial.Serial(com,UARTspeed,timeout=1)
reset()
update(0,0,0)
start=time.time()
for i in range(0,1000):
    update(0,i/100,0)
for i in range(0,1000):
    update(0,(999-i)/100,0)      
end=time.time()
print(end-start)
data.close()
