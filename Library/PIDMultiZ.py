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

class PIDZ:


    def __init__(self, Pz=3.1, Iz=89.7, Dz=0.026, currentTime=None):

        self.Kpz = Pz
        self.Kiz = Iz
        self.Kdz = Dz

        self.sampleTimez = 0.00
        self.currentTime = currentTime if currentTime is not None else time.time()
        self.lastTimez = self.currentTime

        self.clearz()

    def clearz(self):
        self.SetPointz = 0.0
        self.PTermz = 0.0
        self.ITermz = 0.0
        self.DTermz = 0.0
        self.lastErrorz = 0.0

        # Windup Guard
        self.intErrorz = 0.0
        self.windupGuardz = 20.0
        self.outputz = 0.0

    #calculates PID value for given reference feedback
    def updatez(self, feedbackValueZ, currentTime=None):

        errorz = self.SetPointz - feedbackValueZ

        self.currentTime = currentTime if currentTime is not None else time.time()
        deltaTimez = self.currentTime - self.lastTimez
        deltaErrorz = errorz - self.lastErrorz

        if (deltaTimez >= self.sampleTimez):
            self.PTermz = self.Kpz * errorz
            self.ITermz += errorz * deltaTimez

            if (self.ITermz < -self.windupGuardz):
                self.ITermz = -self.windupGuardz
            elif (self.ITermz > self.windupGuardz):
                self.ITermz = self.windupGuardz

            self.DTermz = 0.0
            if deltaTimez > 0:
                self.DTermz = deltaErrorz / deltaTimez

            #Remember last time and last error for next calculation
            self.lastTimez = self.currentTimez
            self.lastErrorz = errorz

            self.outputz = self.PTermz + (self.Kiz * self.ITermz) + (self.Kdz * self.DTermz)

    def setKpz(self, proportionalGainz):
        self.Kpz = proportionalGainz

    def setKiz(self, integralGainz):
        self.Kiz = integralGainz

    def setKdz(self, derivativeGainz):
        self.Kd = derivativeGainz

    def setWindupz(self, windupz):
        self.windupGuardz = windupz

    def setSampleTimez(self, sampleTimez):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sampleTimez = sampleTimez
