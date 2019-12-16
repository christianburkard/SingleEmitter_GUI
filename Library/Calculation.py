"""
Created on 07.10.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
from imutils.video import FPS
from Library.ThreadsLib import FPSOutput
from Library.ThreadsLib import WebcamVideoStream
from imutils.video.pivideostream import PiVideoStream
from Library.Functions import*
from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2
import imutils
import time
import dlib
import csv
import datetime
import time
import threading
import sys
import numpy as np
#from picamera.array import PiRGBArray
#from picamera import PiCamera

# initialization
global meanPartDia
global meanPartDiamm
radius = 10
framenum = 0 # number of frames taken to estimate mean particle diameter
framemax = 1000
realParticleDiameter = 20

#pathWriteDia = 'H:/03_Software/Python/GUI/Logging/MeanParticleDiameter.csv'

# class for general calculations
class Calculation:

    def meanParticleSize(selectedCam):
        x = []
        y = []
        timer = None
        radius = 10
#        framenum = 0 # number of frames taken to estimate mean particle diameter
#        framemax = 1000
#        selectedCam = 2
#        numframes = 200


        if selectedCam == 0:
            print("Your Selection: PiCam")



#           if picamerainst == 1:
            # initialize the camera and grab a reference to the raw camera capture
#            camera = PiCamera()
#            camera.resolution = (624, 480)
#            camera.framerate = 32
#            rawCapture = PiRGBArray(camera, size=(624, 480))
            time.sleep(0.5)
            print("Starting video stream...")

            # allow the camera to warmup
            time.sleep(0.5)
            buffer = 120
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (100, 90, 90)
                print("Set HSV default values ")
#            blackUpper = (80, 80, 80)
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)
            # allow the camera or video file to warm up
            time.sleep(1.0)
            tracker = None
            writer = None
            confdef = 0.4

            # initialization of vectors for safing logging data
            tempPartDiaPixels = np.array([])
            tempFrames = np.array([])
            coordArrayX = np.array([])
            coordArrayY = np.array([])
            radiusArray = np.array([])

#            meanTimeArray = np.append([])
#            elapsedTimeArray = np.append([])

            print("Sampling THREADED frames from `picamera` module...")
            vs = PiVideoStream().start()


            time.sleep(2.0)
            # capture frames from the camera
            fps = FPS().start()

#            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


            for framenum in range (0,framemax):

                frame = vs.read()
                try:
                    frame.shape
                    print("checked for shape".format(frame.shape))
                except AttributeError:
                    print("shape not found")
                    #code to move to next frame


                frame = imutils.resize(frame, width = 400)
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
#                    frame = frame.array
            #   frame = imutils.resize(frame, width=600)
                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                mask1 = cv2.inRange(hsv, blackLower, blackUpper)
                mask2 = cv2.erode(mask1, None, iterations=1)
#                    mask3 = cv2.dilate(mask2, None, iterations=1)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                center = None


            # only proceed if at least one contour was found
                if len(cnts) > 0:
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    if int(M["m00"]) == 0:
                         center = (0, 0)
                    else:
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    PixCoordX = (center[0])
                    PixCoordY = (center[1])
                    print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
            #                pixcoordinate

                    # write data to arduino
#                        writeDataArduino()
                    # only proceed if the radius meets a minimum size
                    if radius > 20:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius))
                    except:
                        print("No radius detected")
                    # Transform pixels to mm
                    PixRadius = radius
                    PixDiameter = 2*PixRadius

                    pts.appendleft(center)

                else:
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    PixRadius = np.nan

            # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
            #        thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
            #        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

                coordArrayX = np.append(coordArrayX,abs(PixCoordX))
                coordArrayY = np.append(coordArrayY,abs(PixCoordY))
                radiusArray = np.append(radiusArray,abs(PixRadius))


                # show the frame
#                mask2 = frame.copy()
                cv2.circle(mask2, (int(320), int(240)), 10, (255, 0, 0), -1)
                cv2.line(mask2, (int(100), int(240)), (int(500), int(240)),(255, 0, 255), 2) #x1,y1,x2,y2
                cv2.line(mask2, (int(320), int(100)), (int(320), int(400)),(255, 0, 255), 2) #x1,y1,x2,y2

                cv2.imshow("Frame", mask2)
#                cv2.imshow("blurred", blurred)
#                cv2.imshow("hsv", hsv)
                key = cv2.waitKey(1) & 0xFF

                # clear the stream in preparation for the next frame
                # safe mean diameter in array with corresponding frame
                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                tempPartDiaPixels = np.append(tempPartDiaPixels,PixDiameter)

#                rawCapture.truncate(0)
                # update counter
                framenum = framenum + 1

                # if the 'q' key is pressed, stop the loop
#                if key == ord("r"):
#    #                cmdStopTimer()
#                    break

                # Update fps counter
                fps.update()


            # stop timer and disp. fps information
            fps.stop()

            #calculate mean diameter
            meanPartDia = sum(tempPartDiaPixels)/frameCounter
            print("Mean particle diameter / px : {:.2f}".format(meanPartDia))

            # transform from px to mm
            meanPartDiamm = meanPartDia/realParticleDiameter
            print("Particle resolution / px/mm : {:.2f}".format(meanPartDiamm))

            # time per frame
            meanTimeFrame = fps.elapsed()/framemax
            print("Frame mean time / s: {:.2f}".format(meanTimeFrame))
            PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, fps.elapsed())

            print("Elapsed time: {:.6f}".format(fps.elapsed()))
            print("Approx. FPS: {:.2f}".format(fps.fps()))

            # mean diameter calculation
            elapsedTimeArray = np.arange(0, fps.elapsed(), meanTimeFrame)
            # mean diameter calculation
            # take time to write data into csv file
            time.sleep(2)
            fpsApprox = fps.fps()
            writeMeanDiameter(tempFrames,coordArrayX,coordArrayY,radiusArray,selectedCam)



            cv2.destroyAllWindows()
            vs.stop()


#        else:
#            print("No PiCamera module installed")


    ####################################################
    #### USB camera setup
    ####################################################

        elif selectedCam == 1:

            print("Your Selection: USB Cam")
            #Start timer
    #        cmdStartTimer()

            # construct the argument parse and parse the arguments
            ap = argparse.ArgumentParser()
            args = vars(ap.parse_args())
            buffer = 120
            # define the lower and upper boundaries of the "black"
            # ball in the HSV color space, then initialize the
            # list of tracked points
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (100, 90, 90)
                print("Set HSV default values ")
#            blackUpper = (100, 90, 90)
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)

            # if a video path was not supplied, grab the reference
            # to the webcam
            if not args.get("video", False):
                print("Starting video stream...")
    #            vs = VideoStream(src=0).start()

                #-threaded videostream
                vs = WebcamVideoStream(src=0).start()

            # otherwise, grab a reference to the video file
            else:
                print(" No video stream possible")

            # allow the camera or video file to warm up
            time.sleep(1.0)
            tracker = None
            writer = None
            confdef = 0.2
            fps = FPSOutput().start()

            # initialization of vectors for safing logging data
            tempPartDiaPixels = np.array([])
            tempFrames = np.array([])
            coordArrayX = np.array([])
            coordArrayY = np.array([])
            radiusArray = np.array([])


            for framenum in range (0,framemax):


                # grab the current frame
                frame = vs.read()

                # handle the frame from VideoCapture or VideoStream
                frame = frame[1] if args.get("video", False) else frame

                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if frame is None:
                    print("No video preview possible")
                    break

                # resize the frame, blur it, and convert it to the HSV
                # color space
                frame = imutils.resize(frame, width=800)
                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # construct a mask for the color "black", then perform
                # a series of dilations and erosions to remove any small
                # blobs left in the mask

                # mask for black color
                mask1 = cv2.inRange(hsv, blackLower, blackUpper)
                mask2 = cv2.erode(mask1, None, iterations=1)

                #Apply median filter
    #            mask3 = cv2.dilate(mask2, None, iterations=1)
    #            mask4 = cv2.medianBlur(mask3,5)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                center = None

                # only proceed if at least one contour was found
                if len(cnts) > 0:
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    if int(M["m00"]) == 0:
                        center = (0, 0)
                    else:
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    PixCoordX = (center[0])
                    PixCoordY = (center[1])
                    print("PiX coordinate: {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
    #                PixelOrbitGraph(PixCoordX,PixCoordY)
    #                plt.pause(1)


                    # only proceed if the radius meets a minimum size
                    if radius > 20:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius), (15, 186, 2), 10)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius))
                    except:
                        print("No radius detected")
                        # Transform pixels to mm
                    PixRadius = radius
                    PixDiameter = 2*PixRadius


#                writeMeanDiameter(PixCoordX,PixCoordY,PixDiameter,framenum)
                    pts.appendleft(center)

                else:
                    PixCoordX = np.nan
                    PixCoordY = np.nan
                    PixRadius = np.nan

                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                coordArrayX = np.append(coordArrayX,abs(PixCoordX))
                coordArrayY = np.append(coordArrayY,abs(PixCoordY))
                radiusArray = np.append(radiusArray,abs(PixRadius))

                # show the frame to our screen
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                # safe mean diameter in array with corresponding frame
                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                tempPartDiaPixels = np.append(tempPartDiaPixels,PixDiameter)


                # update counter
                framenum = framenum + 1

                # if the 'q' key is pressed, stop the loop
#                if key == ord("r"):
#    #                cmdStopTimer()
#                    break

                # Update fps counter
                fps.update()

            # stop timer and disp. fps information
            fps.stop()

            #calculate mean diameter
            meanPartDia = sum(tempPartDiaPixels)/frameCounter
            print("Mean particle diameter / px : {:.2f}".format(meanPartDia))

            # transform from px to mm
            meanPartDiamm = meanPartDia/realParticleDiameter
            print("Particle resolution / px/mm : {:.2f}".format(meanPartDiamm))

            # time per frame
            meanTimeFrame = fps.elapsed()/framemax
            print("Frame mean time / s: {:.2f}".format(meanTimeFrame))
            PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, fps.elapsed())

            print("Elapsed time: {:.6f}".format(fps.elapsed()))
            print("Approx. FPS: {:.2f}".format(fps.fps()))

            #generating elapsed time array
#            counter = 1
#            while counter <= framenum:
#                result = counter
#                elapsedTimeArray = np.append(result_array, result)
#                counter = counter+1
            elapsedTimeArray = np.arange(0, fps.elapsed(), meanTimeFrame)
            # mean diameter calculation
            # take time to write data into csv file
            time.sleep(2)
            fpsApprox = fps.fps()
            writeMeanDiameter(tempFrames,coordArrayX,coordArrayY,radiusArray,selectedCam)

            # delete all variables that are no longer needed
            del tempPartDiaPixels
            del tempFrames
            del coordArrayX
            del coordArrayY
            del radiusArray

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs.stop()

            # otherwise, release the camera
            else:
                vs.release()

            # close all windows
            cv2.destroyAllWindows()




#        print("Mean object diameter: {:.2f}".format(meanObjDia),)
