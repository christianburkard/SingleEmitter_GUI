"""
Created on 11.12.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
from scipy.fftpack import fft
import matplotlib.pyplot as plt
import numpy as np
import imutils
import time
import dlib
from tkinter import filedialog
import tkinter as tk
import time
import pandas as pd
import csv



def showFFT():
    caseText = input("Enter approx frames per second: ")
    approxFPS = int(caseText)

    print("Plotting FFT ...")
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


    sigLen = len(data[:,1])
    fsampling = approxFPS
    timeT = 1/fsampling

    timeSamp = np.linspace(0.0,sigLen*timeT,sigLen)
    freqSamp = np.linspace(0.0, 1.0/(2.0*timeT), sigLen//2)

    zData = (data[:-1,1])
    rData = (data[:-1,2])

    rfft = fft(rData)
    zfft = fft(zData)



    plt.subplot(2, 1, 1)
    plt.plot(freqSamp, 2.0/sigLen*np.abs(rfft[0:sigLen//2]), 'k', color = 'black')
    #plt.title('Position vs time')
    plt.ylabel('R-Amplitude / px')
    locs, labels = plt.xticks()
    plt.xticks(np.arange(0, 201, step=10))
    plt.xlim((1,200))
#    plt.ylim((0,3))

    plt.subplot(2, 1, 2)
    plt.plot(freqSamp, 2.0/sigLen*np.abs(zfft[0:sigLen//2]), 'k', color = 'black')
    plt.xlabel('Frequency f / Hz')
    plt.ylabel('Z-Amplitude / px')
    locs, labels = plt.xticks()
    plt.xticks(np.arange(0, 201, step=10))
    plt.xlim((1,200))
#    plt.ylim((0,abs(3)))

    print("FFT plotting done")