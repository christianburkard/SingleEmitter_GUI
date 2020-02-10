"""
Created on 10.02.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
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

#    caseText = input("Enter approx frames per second: ")
#    approxFPS = int(caseText)

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


ax = plt.axes(projection='3d')

# Data for a three-dimensional line
#zline = np.linspace(0, 15, 1000)
#xline = np.sin(zline)
#yline = np.cos(zline)
#ax.plot3D(xline, yline, zline, 'gray')

# Data for three-dimensional scattered points
zdata = data[:,1]
xdata = data[:,2]
ydata = data[:,3]
ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens');