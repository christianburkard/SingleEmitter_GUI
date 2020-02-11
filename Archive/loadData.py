"""
Created on 20.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
import numpy as np
import tkinter as tk
from tkinter import filedialog, Canvas

print("Choose file ...")
file = filedialog.askopenfilename(initialdir = './Logging')
data = np.load(file)