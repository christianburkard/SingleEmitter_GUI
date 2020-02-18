"""
Created on 11.02.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System
Setup for MultiLev 3D position controlling
"""
# =============================================================================
# MultiLev
# =============================================================================
class MultiLev:
    def objectDetectionMultiLev(self,selected,PIDIncl,reticleIncl):

        if selected == 3:

            time.sleep(0.2)
            #Start timer
    #        cmdStartTimer()

            try:
                frameWidth = int(globFrameWidth)
            except:
                frameWidth = 1024
                print("Default frame width: ", frameWidth)
            try:
                objDia = int(globObjDia)
            except:
                objDia = 2 #in mm
                print("Default object diameter: ", objDia)
            P = 0.5
            I = 1.5
            D = 0.3
            posZ = 0
            initPIDParams(posZ,P,I,D)
            time.sleep(0.1)
            createConfigPID()
            time.sleep(0.1)
            try:
                dataByte1, dataByte2, dataByte3 = defaultLookUp()
            except:
            #read in values from look-up table
                dataByte1, dataByte2, dataByte3 = defLookUpTable()

            # construct the argument parse and parse the arguments
            ap = argparse.ArgumentParser()
            args = vars(ap.parse_args())
            buffer = 120
            # define the lower and upper boundaries of the "black"
            try:
                blackUpper = (int(valueUpperH), int(valueUpperS), int(valueUpperV))
                print("Costumized HSV values entered ...")
            except:
                blackUpper = (150, 120, 120) #default: 120, 90, 90
                print("Set HSV default values ")
            blackLower = (0, 0, 0)
            pts = deque(maxlen=buffer)

            # if a video path was not supplied, grab the reference
            # to the webcam
            if not args.get("video", False):
                print("Starting video stream...")
    #            vs = VideoStream(src=0).start()
                #-threaded videostream
                vs0 = WebcamVideoStream(src=0).start()
                vs1 = WebcamVideoStream(src=1).start()
            # otherwise, grab a reference to the video file
            else:
                print(" No video stream possible")
            # allow the camera or video file to warm up
            time.sleep(0.2)
            deltaWidth0 = 100
            deltaHeight0 = 100
            deltaWidth1 = deltaWidth0
            deltaHeight1 = deltaHeight0
#                height0, width0, channels = frameCropped0.
#                height1, width1, channels = frame1.shape
# =============================================================================
#            Camera parameters
# =============================================================================
            Cmosx = 5.440 #CMOS width in mm
            Cmosy = 3.072 #CMOS height in mm
            PixCalib = 640 #calibration image  width
            PiyCalib = 360 # calibration image height
            fx0 = 507.41 #focal length x px
            fy0 = 505.94 #focal length y px
            Fx0 = fx0*(Cmosx/PixCalib) #focal length x in mm
            Fy0 = fy0*(Cmosy/PiyCalib) #focal length y in mm
            fx1 = 520.28 #focal length x px
            fy1 = 518.81 #focal length y px
            Fx1 = fx1*(Cmosx/PixCalib) #focal length x in mm
            Fy1 = fx1*(Cmosx/PixCalib) #focal length y in mm
            width1 = 1024
            width0 = width1
            height1 = 576
            height0 = height1
            halfCamWidth0 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight0 = height0/2 #y-offset for centering image coordinate system
            halfCamWidth1 = width0/2 #x-offset for centering image coordinate system
            halfCamHeight1 = height1/2 #y-offset for centering image coordinate system
            halfCamWidth0 = int(halfCamWidth0)
            halfCamHeight0 = int(halfCamHeight0)
            halfCamWidth1 = int(halfCamWidth1)
            halfCamHeight1 = int(halfCamHeight1)
            tracker = None
            writer = None
            confdef = 0.2
            framenum = 0
            fps = FPSOutput().start()

# =============================================================================
# #            Array Initialization
# =============================================================================
            vTrans0 = np.array([])
            vTrans1 = np.array([])
            coordArrayX0 = np.array([])
            coordArrayY0 = np.array([])
            radiusArray0 = np.array([])
            coordArrayX1 = np.array([])
            coordArrayY1 = np.array([])
            radiusArray1 = np.array([])
            xArrayTri = np.array([])
            yArrayTri = np.array([])
            zArrayTri = np.array([])
            timeArray = np.array([])
            tempFrames = np.array([])
            tempPartDiaPixels = np.array([])
            pidOutputArray = np.array([])
            PixCoordX0 = 0
            PixCoordX1 = 0
            PixCoordY0 = 0
            PixCoordY1 = 0

            # keep looping
            while True:
                # grab the current frame
                frame0 = vs0.read()
                frame1 = vs1.read()
                # handle the frame from VideoCapture or VideoStream
                frame0 = frame0[1] if args.get("video", False) else frame0
                frame1 = frame1[1] if args.get("video", False) else frame1
                # if we are viewing a video and we did not grab a frame,
                # then we have reached the end of the video
                if frame0 is None:
                    print("No video preview of camera 0 possible")
                    break
                if frame1 is None:
                    print("No video preview of camera 1 possible")
                    break
                # resize the frame, blur it, and convert it to the HSV
                # color space
#                frame = imutils.resize(frame, width=frameWidth)
                frameCropped0 = imutils.resize(frame0, width=frameWidth)
                frameCropped1 = imutils.resize(frame1, width= frameWidth)

                frameCropped10 = frameCropped1[halfCamHeight1-int(deltaHeight1):halfCamHeight1+int(deltaHeight1),halfCamWidth1-int(deltaWidth1):halfCamWidth1+int(deltaWidth1),:]
                frameCropped00 = frameCropped0[halfCamHeight0-int(deltaHeight0):halfCamHeight0+int(deltaHeight0),halfCamWidth0-int(deltaWidth0):halfCamWidth0+int(deltaWidth0),:]
                print(len(frameCropped10))

                blurred0 = cv2.GaussianBlur(frameCropped00, (11, 11), 0)
                blurred1 = cv2.GaussianBlur(frameCropped10, (11, 11), 0)

                # mask for black color
                mask10 = cv2.inRange(blurred0, blackLower, blackUpper)
                mask11 = cv2.inRange(blurred1, blackLower, blackUpper)
                mask20 = cv2.erode(mask10, None, iterations=2)
                mask21 = cv2.erode(mask11, None, iterations=2)

                #Apply median filter
                mask30 = cv2.dilate(mask20, None, iterations=1)
                mask31 = cv2.dilate(mask21, None, iterations=1)
                mask40 = cv2.medianBlur(mask30,5)
                mask41 = cv2.medianBlur(mask31,5)

                # find contours in the mask and initialize the current
                # (x, y) center of the object
                cnts0 = cv2.findContours(mask30.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
#                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
#                    cv2.CHAIN_APPROX_SIMPLE)
                cnts0 = imutils.grab_contours(cnts0)
#                cnts1 = imutils.grab_contours(cnts1)
                center0 = None
                center1 = None

    #            stopTimer()
                print("Contour length: ",len(cnts0))
                # only proceed if at least one contour was found
                if (len(cnts0) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c0 = max(cnts0, key=cv2.contourArea)
                    ((x0, y0), radius0) = cv2.minEnclosingCircle(c0)
                    print("X0",x0)
                    M0 = cv2.moments(c0)
                    if (int(M0["m00"]) != 0) and (radius0 < 150 and radius0 > 20):
                        center0 = (int(M0["m10"] / M0["m00"]), int(M0["m01"] / M0["m00"]))

                        PixCoordX0 = (center0[0]-deltaWidth0)
                        PixCoordY0 = (center0[1]-deltaHeight0)*(-1)
                        radius0 = radius0
                        pixDiameter0 = 2*radius0
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center0 = np.nan
                        PixCoordX0 = np.nan
                        PixCoordY0 = np.nan
                        radius0 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius0 > 20 and radius0 < 150):
#                    if (radius0 > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frameCropped00, (int(center0[0]), int(center0[1])), int(radius0), (15, 186, 2), 10)
                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)


                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius0))
                        PixRadius0 = radius0

                    except:
                        print("No radius detected")

                        PixRadius0 = np.nan
                else:
                    PixCoordX0 = np.nan
                    PixCoordY0 = np.nan
                    radius0 = np.nan
                    PixRadius0 = radius0

                    print("No object detected ...")

                cnts1 = cv2.findContours(mask31.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                if (len(cnts1) > 0):
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c1 = max(cnts1, key=cv2.contourArea)
                    ((x1, y1), radius1) = cv2.minEnclosingCircle(c1)

                    M1 = cv2.moments(c1)
                    if (int(M1["m00"]) != 0) and (radius1 < 150 and radius1 > 20):
                        center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
                        PixCoordX1 = (center1[0]-deltaWidth1)
                        PixCoordY1 = (center1[1]-deltaHeight1)*(-1)
                        radius1 = radius1
                        pixDiameter1 = 2*radius1
                        print("PiX0 coordinate: {:.2f}".format(PixCoordX0), "  PiY0 coordinate: {:.2f}".format(PixCoordY0))
                    else:
                        center1 = np.nan
                        PixCoordX1 = np.nan
                        PixCoordY1 = np.nan
                        radius1 = np.nan
                    # only proceed if the radius meets a minimum size
                    if (radius1 > 20 and radius1 < 150):
#                    if (radius > 0):
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points

                        cv2.circle(frameCropped10, (center1[0], center1[1]), int(radius1), (15, 186, 2), 10)
#                        cv2.circle(frameCropped00, center0, 5, (0, 0, 255), -1)
                        cv2.circle(frameCropped10, center1, 5, (0, 0, 255), -1)

                # update the points queue
                    try:
                        print("Contour radius: {:.2f}".format(radius1))
#                        PixRadius0 = radius0
                        PixRadius1 = radius1

                    except:
                        print("No radius detected")
#
                        PixRadius1 = np.nan
                else:

                    PixCoordX1 = np.nan
                    PixCoordY1 = np.nan
                    radius1 = np.nan
                    PixRadius1 = radius1
#                    print("PiX coordinate {:.2f}".format(PixCoordX), "  PiY coordinate: {:.2f}".format(PixCoordY))
#                    print("Contour radius: {:.2f}".format(radius))
                    print("No object detected ...")
                # loop over the set of tracked points
                for i in range(1, len(pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # show mask images
#                cv2.imshow('HSV', mask1)
#                cv2.imshow('Erode', mask2)
#                cv2.imshow('Dilate', mask3)
#                cv2.imshow("Median Filter", mask4)
                if self.reticleIncl == 1:
#                    height, width, channels = frame.shape
#                    frameCropped0 = frameCropped0.copy()
#                    frameCropped1 = frameCropped1.copy()
                    cv2.circle(frameCropped00, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped00, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped00, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.circle(frameCropped10, (int(deltaWidth0), int(deltaHeight0)), 10, (255, 0, 0), -1)
                    cv2.line(frameCropped10, (int(deltaWidth0-50), int(deltaHeight0)), (int(deltaWidth0+50), int(deltaHeight0)),(255, 0, 255), 4) #x1,y1,x2,y2
                    cv2.line(frameCropped10, (int(deltaWidth0), int(deltaHeight0-50)), (int(deltaWidth0), int(deltaHeight0+50)),(255, 0, 255), 4) #x1,y1,x2,y2

                #geometric properties
                psi0 = 45*(2*math.pi/360)
                psi1 = 45*(2*math.pi/360)
                phi0 = 15.25*(2*math.pi/360)
                phi1 = 8.25*(2*math.pi/360)
                theta0 = 17*(2*math.pi/360)
                theta1 = 17*(2*math.pi/360)


# =============================================================================
# XYZ Coordinates using rot.matrix and angle of attack phi to generate 90 degrees rotated cameras
# =============================================================================
                cphi0 = np.cos(phi0)
                sphi0 = np.sin(phi0)
                cphi1 = np.cos(phi0)
                sphi1 = np.sin(phi0)

                #rotation around phi axis
                Rphi0 = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
                Rphi1 = np.array(((cphi1, sphi1),(-sphi1, cphi1)))
                vphi0 = np.array((PixCoordX0, PixCoordY0))
                vphi1 = np.array((PixCoordX1, PixCoordY1))
                vphi0 = vphi0.dot(Rphi0) #
                vphi1 = vphi1.dot(Rphi1)
                print("V0",vphi0)
                print("V1",vphi1)

                Rglobal = np.array(((cphi0, -sphi0),(sphi0, cphi0)))
                Rglobal3D = np.array(((1, 0, 0), (0, np.cos(psi0), -np.sin(psi0)), (0, np.sin(psi0), np.cos(psi0))))

                vpsi0 = np.array


                xTriLocal = (vphi0[0]+vphi1[0])/2
                yTriLocal = vphi0[1]
                zTriLocal = vphi1[1]

                vGlobal = np.array((xTriLocal, yTriLocal, zTriLocal))
                vGlobal = vGlobal.dot(Rglobal3D)


                xTri = vGlobal[0]
                zTri = vGlobal[1]
                yTri = vGlobal[2]

                print('X',xTri)
                print('Y',yTri)
                print('Z',zTri)

                # show the frame to our screen
#                rotated=cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Cropped Frame 0", frameCropped00)
                cv2.imshow("Cropped Frame 1", frameCropped10)
                cv2.imshow("Mask30", mask30)
                cv2.imshow("Mask31", mask31)
                key = cv2.waitKey(1) & 0xFF

                if self.PIDIncl == 1:
                    pid = readConfigPID()
                    pid.SetPoint = float(spinBoxVal)
                    pid.setSampleTime(0.001)
                    pid.setKp(1.1) #default: 3.1
                    pid.setKi(200) #default: 89.7
                    pid.setKd(0.025) #default: 0.025
#                    pid.update(PixCoordY)
                    pid.update(zTri)
                    pidOutputVal = float(pid.output)
                    print("PID output",pid.output)

                else:
                    pidOutputVal = np.nan

#                print("PID output: ",pid.output)
                # if the 'q' key is pressed, stop the loop
                if key == ord("r"):
    #                cmdStopTimer()
                    break
                elif selectedStopAll == 1:
                    break

#                try:
#                    spinBoxVal = int(spinBoxVal)
#                except:
#                    spinBoxVal = 0 #in mm

                #open-loop control adjusted via the user interface
                objZPos = setObjPos(self, spinBoxVal)
                print("Obj Z position value: ",objZPos)
                if (objZPos >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(objZPos)]
                    byte2 = dataByte2[int(objZPos)]
                    byte3 = dataByte3[int(objZPos)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)
                elif (objZPos >= 0 and self.PIDIncl == 0):
                    byte1 = dataByte1[int(720 + objZPos)]
                    byte2 = dataByte2[int(720 + objZPos)]
                    byte3 = dataByte3[int(720 + objZPos)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)


#                #open-loop control adjusted via the user interface
                if (pidOutputVal <= 0 and self.PIDIncl == 1):
                    objZPosCL = (pidOutputVal/15)
                    byte1 = dataByte1[int(719 + objZPosCL)]
                    byte2 = dataByte2[int(719 + objZPosCL)]
                    byte3 = dataByte3[int(719 + objZPosCL)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                elif (pidOutputVal > 0 and self.PIDIncl == 1):
                    objZPosCL = (pidOutputVal/15)
                    byte1 = dataByte1[int(objZPosCL)]
                    byte2 = dataByte2[int(objZPosCL)]
                    byte3 = dataByte3[int(objZPosCL)]
                    values = bytearray([byte1, byte2, byte3])
                    serialObject.write(values)
                    print("Serial Values: ",byte1)

                frameCounter = framenum + 1
                tempFrames = np.append(tempFrames,frameCounter)
                try:
                    tempPartDiaPixels = np.append(tempPartDiaPixels,pixDiameter0)
                except:
                    None
                pidOutputArray = np.append(pidOutputArray,pidOutputVal)
                coordArrayX0 = np.append(coordArrayX0,abs(PixCoordX0))
                coordArrayY0 = np.append(coordArrayY0,abs(PixCoordY0))
                radiusArray0 = np.append(radiusArray0,abs(PixRadius0))
                coordArrayX1 = np.append(coordArrayX1,abs(PixCoordX1))
                coordArrayY1 = np.append(coordArrayY1,abs(PixCoordY1))
                radiusArray1 = np.append(radiusArray1,abs(PixRadius1))
                xArrayTri = np.append(xArrayTri,(xTri))
                yArrayTri = np.append(yArrayTri,(yTri))
                zArrayTri = np.append(zArrayTri,(zTri))
                timeArray = np.append(timeArray,time.time()) # time in seconds
                # update counter
                framenum = framenum + 1
                print("Framenumber: ",framenum)
#
#                print("PID Array length: ",len(pidOutputArray))
#                print("coordArrayY: ",len(coordArrayY))
#                print("Time array: ",len(timeArray))
                printCoordsGui(self, xTri, yTri, zTri)
                window.update()

#                time.sleep(0.5)
                # Update fps counter
                fps.update()


            # stop timer and disp. fps information
            fps.stop()
            fpsVar = float((fps.fps()))
            print("Elapsed time: {:.2f}".format(fps.elapsed()))
            print("Approx. FPS: {:.2f}".format(fps.fps()))

            #calculate mean diameter
            meanPartDia = sum(tempPartDiaPixels)/frameCounter
            print("Mean particle diameter / px : {:.2f}".format(meanPartDia))

            # transform from px to mm
            meanPartDiamm = meanPartDia/objDia
            print("Particle resolution / px/mm : {:.2f}".format(meanPartDiamm))

            # time per frame
            meanTimeFrame = fps.elapsed()/framenum
            print("Frame mean time / s: {:.2f}".format(meanTimeFrame))
#            PixelDiavsTime(tempPartDiaPixels, meanTimeFrame, fps.elapsed())

            writePixelPositionPCCam1(timeArray,coordArrayX0,coordArrayY0,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPositionPCCam2(timeArray,coordArrayX1,coordArrayY1,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePixelPosition3D(timeArray,xArrayTri,yArrayTri,zArrayTri,radiusArray0,framenum,fpsVar,self.PIDIncl)
            writePIDOutput(timeArray,coordArrayY0,pidOutputArray)

            # if we are not using a video file, stop the camera video stream
            if not args.get("video", False):
                vs0.stop()
                vs1.stop()

            # otherwise, release the camera
            else:
                vs0.release()
                vs1.release()
            # close all windows
            cv2.destroyAllWindows()