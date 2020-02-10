"""
Created on 20.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
from __future__ import print_function
from imutils.video import VideoStream
from imutils.video import FPS
from imutils.video.pivideostream import PiVideoStream
import time
import cv2
import numpy as np
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration
from datetime import datetime
import argparse
import time
import dlib
import csv
import datetime
import time
import imutils
from datetime import datetime

# Depth map default preset
SWS = 5
PFS = 5
PFC = 29
MDS = -30
NOD = 160
TTH = 100
UR = 10
SR = 14
SPWS = 100

# Camera settimgs
cam_width = 360
cam_height = 240
framewidth = 360
# Final image capture settings
scale_ratio = 1.0

# Camera resolution height must be dividable by 16, and width by 32
cam_width = int((cam_width+31)/32)*32
cam_height = int((cam_height+15)/16)*16
print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

# Initialize the camera
#camera = PiCamera(stereo_mode='side-by-side',stereo_decimate=False)
#camera.resolution=(cam_width, cam_height)
#camera.framerate = 20
#camera.hflip = True
ap = argparse.ArgumentParser()
args = vars(ap.parse_args())
if not args.get("video", False):
    print("Starting video stream...")
    time.sleep(0.2)
    vs1 = VideoStream(src=0).start()
    vs2 = VideoStream(src=1).start()
#    vs = WebcamVideoStream(src=0).start()


# Implementing calibration data
print('Read calibration data and rectifying stereo pair...')
calibration = StereoCalibration(input_folder='calibData')

# Initialize interface windows
cv2.namedWindow("Image")
#cv2.moveWindow("Image", 50,100)
cv2.namedWindow("left")
#cv2.moveWindow("left", 450,100)
cv2.namedWindow("right")
#cv2.moveWindow("right", 850,100)


disparity = np.zeros((img_width, img_height), np.uint8)
sbm = cv2.StereoBM_create(numDisparities=0, blockSize=21)

def stereo_depth_map(rectified_pair):
    dmLeft = rectified_pair[0]
    dmRight = rectified_pair[1]
    disparity = sbm.compute(dmLeft, dmRight)
    local_max = disparity.max()
    local_min = disparity.min()
    disparity_grayscale = (disparity-local_min)*(65535.0/(local_max-local_min))
    disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0/65535.0))
    disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
    cv2.imshow("Image", disparity_color)
    key = cv2.waitKey(1) & 0xFF

    return disparity_color

def load_map_settings( fName ):
    global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
    print('Loading parameters from file...')
    f=open(fName, 'r')
    data = json.load(f)
    SWS=data['SADWindowSize']
    PFS=data['preFilterSize']
    PFC=data['preFilterCap']
    MDS=data['minDisparity']
    NOD=data['numberOfDisparities']
    TTH=data['textureThreshold']
    UR=data['uniquenessRatio']
    SR=data['speckleRange']
    SPWS=data['speckleWindowSize']
    #sbm.setSADWindowSize(SWS)
    sbm.setPreFilterType(1)
    sbm.setPreFilterSize(PFS)
    sbm.setPreFilterCap(PFC)
    sbm.setMinDisparity(MDS)
    sbm.setNumDisparities(NOD)
    sbm.setTextureThreshold(TTH)
    sbm.setUniquenessRatio(UR)
    sbm.setSpeckleRange(SR)
    sbm.setSpeckleWindowSize(SPWS)
    f.close()
    print ('Parameters loaded from file '+fName)


load_map_settings ("../Data/3dmap_set.txt")

# capture frames from the camera
#for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
while True:
    frame1 = vs1.read()
    frame2 = vs2.read()
    # handle the frame from VideoCapture or VideoStream
    frame1 = frame1[1] if args.get("video", False) else frame1
    frame1 = imutils.resize(frame1, width=framewidth)
#    cv2.imshow("pair1", frame1)
    frame2 = frame2[1] if args.get("video", False) else frame2
    frame2 = imutils.resize(frame2, width=framewidth)
#    cv2.imshow("pair2", frame2)
#    frame2 = frame2[0:200,100:300,:]
    t1 = datetime.now()
    pair_img1 = cv2.cvtColor (frame1, cv2.COLOR_BGR2GRAY)
    pair_img2 = cv2.cvtColor (frame2, cv2.COLOR_BGR2GRAY)

    imgLeft = pair_img1 [0:img_height,0:img_width] #Y+H and X+W
    imgRight = pair_img2 [0:img_height,0:img_width] #Y+H and X+W
    rectified_pair = calibration.rectify((imgLeft, imgRight))
    disparity = stereo_depth_map(rectified_pair)
    # show the frame
    cv2.imshow("left", imgLeft)
    cv2.imshow("right", imgRight)
    t2 = datetime.now()
    print ("DM build time: " + str(t2-t1))
    key = cv2.waitKey(1) & 0xFF
    if key == ord("r"):
        break

cv2.destroyAllWindows()
time.sleep(0.2)
print("Camera connection closed ...")
vs1.stop()
vs2.stop()