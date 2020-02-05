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

class PID:


    def __init__(self, P=3.1, I=89.7, D=0.026, currentTime=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sampleTime = 0.00
        self.currentTime = currentTime if currentTime is not None else time.time()
        self.lastTime = self.currentTime

        self.clear()

    def clear(self):
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.lastError = 0.0

        # Windup Guard
        self.intError = 0.0
        self.windupGuard = 20.0

        self.output = 0.0

    #calculates PID value for given reference feedback
    def update(self, feedbackValue, currentTime=None):

        error = self.SetPoint - feedbackValue

        self.currentTime = currentTime if currentTime is not None else time.time()
        deltaTime = self.currentTime - self.lastTime
        deltaError = error - self.lastError

        if (deltaTime >= self.sampleTime):
            self.PTerm = self.Kp * error
            self.ITerm += error * deltaTime

            if (self.ITerm < -self.windupGuard):
                self.ITerm = -self.windupGuard
            elif (self.ITerm > self.windupGuard):
                self.ITerm = self.windupGuard

            self.DTerm = 0.0
            if deltaTime > 0:
                self.DTerm = deltaError / deltaTime

            #Remember last time and last error for next calculation
            self.lastTime = self.currentTime
            self.lastError = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportionalGain):
        self.Kp = proportionalGain

    def setKi(self, integralGain):
        self.Ki = integralGain

    def setKd(self, derivativeGain):
        self.Kd = derivativeGain

    def setWindup(self, windup):
        self.windupGuard = windup

    def setSampleTime(self, sampleTime):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sampleTime = sampleTime
