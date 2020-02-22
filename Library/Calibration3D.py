"""
Created on 20.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
import os
import cv2
import numpy as np
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration
from stereovision.exceptions import ChessboardNotFoundError


def start3dCalibration():
    # Global variables preset
    total_photos = 30
    photo_width = 1024
    photo_height = 512
    img_width = 1024
    img_height = 512
    image_size = (img_width,img_height)

    # Chessboard parameters
    rows = 6
    columns = 9
    square_size = 2.5


    calibrator = StereoCalibrator(rows, columns, square_size, image_size)
    photo_counter = 0
    print ('Start cycle')

    while photo_counter != total_photos:
    #while photo_counter >= 3 and  photo_counter < 4:

      photo_counter = photo_counter + 1
      print ('Import pair No ' + str(photo_counter))
    #  leftName = './pairsNew/scene1_512x256_'+str(photo_counter).zfill(2)+'.png'
    #  rightName = './pairsNew/scene2_512x256_'+str(photo_counter).zfill(2)+'.png'
      leftName = './Images/Chessboard/scene1_1024x512_'+str(photo_counter).zfill(2)+'.png'
      rightName = './Images/Chessboard/scene2_1024x512_'+str(photo_counter).zfill(2)+'.png'
      if os.path.isfile(leftName) and os.path.isfile(rightName):
          imgLeft = cv2.imread(leftName,1)
          imgRight = cv2.imread(rightName,1)
          try:
            calibrator._get_corners(imgLeft)
            calibrator._get_corners(imgRight)
          except ChessboardNotFoundError as error:
            print (error)
            print ("Pair No "+ str(photo_counter) + " ignored")
          else:
            calibrator.add_corners((imgLeft, imgRight), True)
    print ('End cycle')


    print ('Starting calibration... It can take several minutes!')
    calibration = calibrator.calibrate_cameras()
    calibration.export('calibData')
    print ('Calibration complete!')


    # Lets rectify and show last pair after  calibration
    calibration = StereoCalibration(input_folder='calibData')
    rectified_pair = calibration.rectify((imgLeft, imgRight))

    cv2.imshow('Left CALIBRATED', rectified_pair[0])
    cv2.imshow('Right CALIBRATED', rectified_pair[1])
    cv2.imwrite("rectifyed_left.jpg",rectified_pair[0])
    cv2.imwrite("rectifyed_right.jpg",rectified_pair[1])
