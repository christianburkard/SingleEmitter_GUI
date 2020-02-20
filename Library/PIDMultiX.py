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

class PIDX:


    def __init__(self, Px=3.1, Ix=89.7, Dx=0.026, currentTime=None):

        self.Kpx = Px
        self.Kix = Ix
        self.Kdx = Dx

        self.sampleTimex = 0.00
        self.currentTime = currentTime if currentTime is not None else time.time()
        self.lastTimex = self.currentTime

        self.clearx()

    def clearx(self):
        self.SetPointx = 0.0
        self.PTermx = 0.0
        self.ITermx = 0.0
        self.DTermx = 0.0
        self.lastErrorx = 0.0

        # Windup Guard
        self.intErrorx = 0.0
        self.windupGuardx = 20.0
        self.outputx = 0.0

    #calculates PID value for given reference feedback
    def updatex(self, feedbackValueX, currentTime=None):

        errorx = self.SetPointx - feedbackValueX

        self.currentTime = currentTime if currentTime is not None else time.time()
        deltaTimex = self.currentTime - self.lastTimex
        deltaErrorx = errorx - self.lastErrorx

        if (deltaTimex >= self.sampleTimex):
            self.PTermx = self.Kpx * errorx
            self.ITermx += errorx * deltaTimex

            if (self.ITermx < -self.windupGuardx):
                self.ITermx = -self.windupGuardx
            elif (self.ITermx > self.windupGuardx):
                self.ITermx = self.windupGuardx

            self.DTermx = 0.0
            if deltaTimex > 0:
                self.DTermx = deltaErrorx / deltaTimex

            #Remember last time and last error for next calculation
            self.lastTimex = self.currentTime
            self.lastErrorx = errorx

            self.outputx = self.PTermx + (self.Kix * self.ITermx) + (self.Kdx * self.DTermx)

    def setKpx(self, proportionalGainx):
        self.Kpx = proportionalGainx

    def setKix(self, integralGainx):
        self.Kix = integralGainx

    def setKdx(self, derivativeGainx):
        self.Kdx = derivativeGainx

    def setWindupx(self, windupx):
        self.windupGuardx = windupx

    def setSampleTime(self, sampleTime):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sampleTime = sampleTime
