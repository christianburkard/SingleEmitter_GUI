"""
Created on 04.11.2019

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

Script for calculating image distortion in 2 dimensions for a single image (or a given set of images).
Output values are the camera matrix and calibrated imges according to the cv2.undistort function as well as
a remapped routine. Input images need to be a calibrated chessboards from different angles. At least 10 images are needed.
"""

import numpy as np
import cv2
import glob
import time
from PIL import Image
from tkinter import filedialog, Canvas
import os

framenumber = 0
imageList = []

def startTimer():
    global starttimer
    starttimer = time.time()
    print("Timer Started")


def stopTimer():
    global endtimer
    endtimer = time.time()
    print("Execution time /s: {:.9f} ".format(endtimer - starttimer))
    print("Timer Stopped")


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*2.5 #multiplier for distance in mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

print("Choose folder with chessboard images ...")
imPath =  filedialog.askdirectory()
dataPath = os.path.join(imPath,'*')
dataPath = glob.glob(dataPath)
data =[]
startTimer()

for fname in dataPath:
    img = cv2.imread(fname)
#    cv2.imshow("Frame",img)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None) #default (7,6)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
#        framenumber = framenumber + 1
    else:
        print("No edges detected ...")
        print(fname)

stopTimer()
cv2.destroyAllWindows()


#Calibration, returns camera matrix, distortion coefficients, rotation and translation vectors
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
#ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape,None,None)

print("Choose folder for saving camera intrinsic parameters ...")
file = filedialog.askdirectory(initialdir = './Logging')
folderName1 = os.path.join(file, 'ret_mat' + ".npy")
folderName2 = os.path.join(file, 'mtx_mat' + ".npy")
folderName3 = os.path.join(file, 'dist_mat' + ".npy")
folderName4 = os.path.join(file, 'rvecs_mat' + ".npy")
folderName5 = os.path.join(file, 'tvecs_mat' + ".npy")
np.save(folderName1, ret)
np.save(folderName2, mtx)
np.save(folderName3, dist)
np.save(folderName4, rvecs)
np.save(folderName5, tvecs)



#def _interact_with_folder(self, output_folder, action):
#    """
#    Export/import matrices as *.npy files to/from an output folder.
#
#    ``action`` is a string. It determines whether the method reads or writes
#    to disk. It must have one of the following values: ('r', 'w').
#    """
#    if not action in ('r', 'w'):
#        raise ValueError("action must be either 'r' or 'w'.")
#    for key, item in self.__dict__.items():
#        if isinstance(item, dict):
#            for side in ("left", "right"):
#                filename = os.path.join(output_folder,
#                                        "{}_{}.npy".format(key, side))
#                if action == 'w':
#                    np.save(filename, self.__dict__[key][side])
#                else:
#                    self.__dict__[key][side] = np.load(filename)
#        else:
#            filename = os.path.join(output_folder, "{}.npy".format(key))
#            if action == 'w':
#                np.save(filename, self.__dict__[key])
#            else:
#                self.__dict__[key] = np.load(filename)

#Undistortion
print("Choose image for undistortion and rectification ...")
filePath = filedialog.askopenfilename()
img = cv2.imread(filePath)
h,w = img.shape[:2]
newcameramtx, roi =cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

# undistort with cv2.undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x_u,y_u,w_u,h_u = roi
dst_u = dst[y_u:y_u+h_u, x_u:x_u+w_u]
cv2.imwrite('./Images/calibresultundistorted.png',dst_u)

# undistort using remapping
mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
dst_rem = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

# crop the image
x_rem,y_rem,w_rem,h_rem = roi
dst_rem = dst[y_rem:y_rem+h_rem, x_rem:x_rem+w_rem]
cv2.imwrite('./Images/calibresults_remapped.png',dst_rem)
