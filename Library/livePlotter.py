"""
Created on 11.02.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
#import tkinter as tk
#from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#from matplotlib.figure import Figure
#from math import cos, sin


# Use python 3 consistent printing and unicode
from __future__ import print_function
from __future__ import unicode_literals

import sys

# This code was written for Python 2.7, and has not been tested on Python 3
#     but may work, so the following section addresses some obvious
#    Python 2 vs 3 compatibility issues
if (sys.version_info < (3, 0)):  # Python 2
    from Tkinter import Tk, Frame, Button
else:  # Python 3
    from tkinter import Tk, Frame, Button

ticks = []  # initialize timer store for speed optimization
from time import clock
#ticks.append(clock()) # time at start

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from numpy import random, sin, exp
from matplotlib import pyplot

#ticks.append(clock()) # time at start of plotting

root = Tk()
root.wm_title("Live Plot") # title for window

class RealTimePlot(Frame) :
    def __init__(self) :
        Frame.__init__(self)
        self.top=Frame()
        self.top.grid()
        self.top.update_idletasks()

        self.x = 0
        self.i = 0
        self.delta_i = 1
        self.update = 10
        # to speed things up, never plot more than n_points on screen
        self.n_points = 500
        self.n_data = 10000     # maximum number of points to acquire
        self.xy_data = []

        self.figure = pyplot.figure()
        # figsize (w,h tuple in inches) dpi (dots per inch)
        self.figure.set_size_inches((4,3), forward=True)
        self.subplot = self.figure.add_subplot(211)

        self.line, = self.subplot.plot([],[]) # initialize line to be drawn
        # Note: The comma after line is because the right hand size returns
        #   a one element list, and we want to unpack that single element
        #   into line, not assign the list to line.
        self.text = pyplot.figtext(0.05,0.25,"") # initialize text section

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.top)
        self.canvas.get_tk_widget().grid(row=3,column=0,columnspan=3)

        self.button_text = ['Start','Pause']
        self.buttons = [None] * len(self.button_text)

        for button_index in range(len(self.button_text)) :
            button_id = Button(self.top,text=self.button_text[button_index])
            button_id.grid(row=0, column=button_index)
            self.buttons[button_index] = button_id

            def button_handler(event, self=self, button=button_index):
                return self.service_buttons(button)

            button_id.bind("<Button-1>", button_handler)

    # buttons can be used to start and pause plotting
    def service_buttons(self, toolbar_index):
        if toolbar_index == 0 :
            self.stop = False
            self.plotter()
        else:
            self.stop = True

    # while in start, check if stop is clicked, if not, call blink recursivly
    def plotter(self):
        if not self.stop :
            self.x += 0.1
            self.y = exp(-self.i*0.005)*sin(self.x)+0.1*random.randn()
            self.xy_data += [[self.x,self.y]]
            # If there are many data points, it is a waste of time to plot all
            #   of them once the screen resolution is reached,
            #   so when the maximum number of points is reached,
            #   halve the number of points plotted. This is repeated
            #   every time the number of data points has doubled.
            if self.i == self.n_points :
                self.n_points *= 2
                # frequency of plotted points
                self.delta_i *= self.n_points/self.i
                self.update = max(self.delta_i, self.update)
                print("updating n_rescale = ",\
            	    self.n_points, self.update, self.delta_i)
            # drawing the canvas takes most of the CPU time, so only update plot
            #   every so often
            if self.i == self.n_data-1 or not (self.i % self.update)  :
                # remove previous version of line plot
                self.subplot.lines.remove(self.line)
                self.figure.texts.remove(self.text)
                self.line, = self.subplot.plot(
                            [row[0] for row in self.xy_data[0::self.delta_i]],
                                [row[1] for row in self.xy_data[0::self.delta_i]],
                                color="blue")
                self.text = pyplot.figtext(0.05,0.25,
                                  "Point # " + str(self.i+1) +
                                  "\nx,y = " + str(self.x) + ", " + str(self.y))
            self.i += 1
            # stop if desired number of points plotted
            if self.i == self.n_data :
                self.service_buttons(1)
            self.canvas.draw()
            self.canvas.get_tk_widget().update_idletasks()
            self.after(2,self.plotter)

RealTimePlot().mainloop()

#ticks.append(clock()) # time at end

# Print timings
#print( "Initialization took : {} seconds".format(ticks[1]-ticks[0]) )
#print( "Drawing took        : {} seconds".format(ticks[2]-ticks[1]) )



#class App:
#    def __init__(self, master):
#        # Create a container
#        frame = tk.Frame(master)
#        # Create 2 buttons
#        self.button_left = tk.Button(frame,text="< Decrease Slope",
#                                        command=self.decrease)
#        self.button_left.pack(side="left")
#        self.button_right = tk.Button(frame,text="Increase Slope >",
#                                        command=self.increase)
#        self.button_right.pack(side="left")
#
#        fig = Figure()
#        ax = fig.add_subplot(111)
#        self.line, = ax.plot([x/0.5 for x in range(20)])
#        self.canvas = FigureCanvasTkAgg(fig,master=master)
#        self.canvas.draw()
#        self.canvas.get_tk_widget().pack(side='top', fill='both', expand=1)
#        frame.pack()
#
#    def decrease(self):
#        x, y = self.line.get_data()
#        self.line.set_ydata(y+[cos(xx) for xx in x])
#        self.canvas.draw()
#
#    def increase(self):
#        x, y = self.line.get_data()
#        self.line.set_ydata(y + 0.2 * x)
#        self.canvas.draw()
#
#root = tk.Tk()
#app = App(root)
#root.mainloop()





#class ServoDrive(object):
#    # simulate values
#    def getVelocity(self): return random.randint(0,50)
#    def getTorque(self): return random.randint(100,150)
#
#class Example(tk.Frame):
#    def __init__(self, *args, **kwargs):
#        tk.Frame.__init__(self, *args, **kwargs)
#        self.servo = ServoDrive()
#        self.canvas = tk.Canvas(self, background="black")
#        self.canvas.pack(side="top", fill="both", expand=True)
#
#        # create lines for velocity and torque
#        self.velocity_line = self.canvas.create_line(0,0,0,0, fill="red")
#        self.torque_line = self.canvas.create_line(0,0,0,0, fill="blue")
#
#        # start the update process
#        self.update_plot()
#
#    def update_plot(self):
#        v = self.servo.getVelocity()
#        t = self.servo.getTorque()
#        self.add_point(self.velocity_line, v)
#        self.add_point(self.torque_line, t)
#        self.canvas.xview_moveto(1.0)
#        self.after(10, self.update_plot)
#
#    def add_point(self, line, y):
#        coords = self.canvas.coords(line)
#        x = coords[-2] + 1
#        coords.append(x)
#        coords.append(y)
#        coords = coords[-1200:] # keep # of points to a manageable size
#        self.canvas.coords(line, *coords)
#        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
#
#if __name__ == "__main__":
#    root = tk.Tk()
#    Example(root).pack(side="top", fill="both", expand=True)
#    root.mainloop()

