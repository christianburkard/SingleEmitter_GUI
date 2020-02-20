"""
Created on 18.12.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System
PID control algorithm.
Example class call:

pid = readConfigPID()
pid.SetPoint = float(set point value)
pid.setSampleTime(0.0001)
pid.setKp(3.1) #default: 3.1
pid.setKi(120) #default: 89.7
pid.setKd(0.025) #default: 0.025
pid.update(control variable)
pidOutputVal = float(pid.output)

"""

import time

class PIDY:


    def __init__(self, Py=3.1, Iy=89.7, Dy=0.026, currentTime=None):

        self.Kpy = Py
        self.Kiy = Iy
        self.Kdy = Dy

        self.sampleTimey = 0.00
        self.currentTime = currentTime if currentTime is not None else time.time()
        self.lastTimey = self.currentTime

        self.cleary()

    def cleary(self):
        self.SetPointy = 0.0
        self.PTermy = 0.0
        self.ITermy = 0.0
        self.DTermy = 0.0
        self.lastErrory = 0.0

        # Windup Guard
        self.intErrory = 0.0
        self.windupGuardy = 20.0
        self.outputy = 0.0

    #calculates PID value for given reference feedback
    def updatey(self, feedbackValueY, currentTime=None):

        errory = self.SetPointy - feedbackValueY

        self.currentTime = currentTime if currentTime is not None else time.time()
        deltaTimey = self.currentTime - self.lastTimey
        deltaErrory = errory - self.lastErrory

        if (deltaTimey >= self.sampleTimey):
            self.PTermy = self.Kpy * errory
            self.ITermy += errory * deltaTimey

            if (self.ITermy < -self.windupGuardy):
                self.ITermy = -self.windupGuardy
            elif (self.ITermy > self.windupGuardy):
                self.ITermy = self.windupGuardy

            self.DTermy = 0.0
            if deltaTimey > 0:
                self.DTermy = deltaErrory / deltaTimey

            #Remember last time and last error for next calculation
            self.lastTimey = self.currentTime
            self.lastErrory = errory

            self.outputy = self.PTermy + (self.Kiy * self.ITermy) + (self.Kdy * self.DTermy)

    def setKpy(self, proportionalGainy):
        self.Kpy = proportionalGainy

    def setKiy(self, integralGainy):
        self.Kiy = integralGainy

    def setKdy(self, derivativeGainy):
        self.Kdy = derivativeGainy

    def setWindupy(self, windupy):
        self.windupGuardy = windupy

    def setSampleTimey(self, sampleTimey):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sampleTimey = sampleTimey
