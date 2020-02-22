"""
Created on 20.01.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""
import cv2
import os
#from picamera.array import PiRGBArray
#from picamera import PiCamera
from matplotlib import colors
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration

#camCalib
#####################################
#Functions
#####################################
class depthMapTuning():

    def load_map_settings(event ):
        global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
        loading_settings = 1
        fName = './Data/3dmap_set.txt'
        print('Loading parameters from file...')
        buttonl.label.set_text ("Loading...")
        f=open(fName, 'r')
        data = json.load(f)
        sSWS.set_val(data['SADWindowSize'])
        sPFS.set_val(data['preFilterSize'])
        sPFC.set_val(data['preFilterCap'])
        sMDS.set_val(data['minDisparity'])
        sNOD.set_val(data['numberOfDisparities'])
        sTTH.set_val(data['textureThreshold'])
        sUR.set_val(data['uniquenessRatio'])
        sSR.set_val(data['speckleRange'])
        sSPWS.set_val(data['speckleWindowSize'])
        f.close()
        buttonl.label.set_text ("Load settings")
        print ('Parameters loaded from file '+fName)
        print ('Redrawing depth map with loaded parameters...')
        loading_settings = 0
        update(0)
        print ('Done!')


    def stereo_depth_map( rectified_pair):
        numDisparities = 12 #default = 16
        blockSize = 11 #default = 15
        print ('SWS='+str(SWS)+' PFS='+str(PFS)+' PFC='+str(PFC)+' MDS='+\
               str(MDS)+' NOD='+str(NOD)+' TTH='+str(TTH))
        print (' UR='+str(UR)+' SR='+str(SR)+' SPWS='+str(SPWS))
        c, r = rectified_pair[0].shape
        disparity = np.zeros((c, r), np.uint8)
        sbm = cv2.StereoBM_create(numDisparities=numDisparities, blockSize=blockSize)
    #    sbm.SADWindowSize = SWS
        sbm.setPreFilterType(1)
        sbm.setPreFilterSize(PFS)
        sbm.setPreFilterCap(PFC)
        sbm.setMinDisparity(MDS)
        sbm.setNumDisparities(NOD)
        sbm.setTextureThreshold(TTH)
        sbm.setUniquenessRatio(UR)
        sbm.setSpeckleRange(SR)
        sbm.setSpeckleWindowSize(SPWS)
        dmLeft = rectified_pair[0]
        dmRight = rectified_pair[1]
    #    cv2.FindStereoCorrespondenceBM(dmLeft, dmRight, disparity, sbm)
        disparity = sbm.compute(dmLeft, dmRight)
    #    disparity_visual = cv2.CreateMat(c, r, cv.CV_8U)
        local_max = disparity.max()
        local_min = disparity.min()
        print ("MAX " + str(local_max))
        print ("MIN " + str(local_min))
        disparity_visual = (disparity-local_min)*(1.0/(local_max-local_min))
        local_max = disparity_visual.max()
        local_min = disparity_visual.min()
        print ("MAX " + str(local_max))
        print ("MIN " + str(local_min))
        #cv.Normalize(disparity, disparity_visual, 0, 255, cv.CV_MINMAX)
        #disparity_visual = np.array(disparity_visual)
        return disparity_visual

    # Update depth map parameters and redraw
    def update(self, val):
        global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS
        SWS = int(sSWS.val/2)*2+1 #convert to ODD
        PFS = int(sPFS.val/2)*2+1
        PFC = int(sPFC.val/2)*2+1
        MDS = int(sMDS.val)
        NOD = int(sNOD.val/16)*16
        TTH = int(sTTH.val)
        UR = int(sUR.val)
        SR = int(sSR.val)
        SPWS= int(sSPWS.val)
        if ( loading_settings==0 ):
            print ('Rebuilding depth map')
            disparity = stereo_depth_map(rectified_pair)
            dmObject.set_data(disparity)
            print ('Redraw depth map')
            plt.draw()

    def save_map_settings(event ):
        buttons.label.set_text ("Saving...")
        print('Saving to file...')
        result = json.dumps({'SADWindowSize':SWS, 'preFilterSize':PFS, 'preFilterCap':PFC, \
                 'minDisparity':MDS, 'numberOfDisparities':NOD, 'textureThreshold':TTH, \
                 'uniquenessRatio':UR, 'speckleRange':SR, 'speckleWindowSize':SPWS},\
                 sort_keys=True, indent=4, separators=(',',':'))
        fName = '3dmap_set.txt'
        f = open (str(fName), 'w')
        f.write(result)
        f.close()
        buttons.label.set_text ("Save to file")
        print ('Settings saved to file '+fName)



##########################################
    #Tuning
##########################################
    def startdepthMapTuning(self):
    #    while True:
        # Global variables preset
        print('Current directory:',os.getcwd())
        imageToDisp = './Images/Spheres/Frame2.png'
        photo_width = 1024
        photo_height = 512
        image_height = 512
        image_width = 1024
        image_size = (image_width,image_height)

        global numDisparities
        global blockSize
        numDisparities = 12 #default = 16
        blockSize = 11 #default = 15

        if os.path.isfile(imageToDisp) == False:
            print ('Can not read image from file \"'+imageToDisp+'\"')
            exit(0)

        pair_img = cv2.imread(imageToDisp,0)
        # Read image and split it in a stereo pair
        print('Read and split image...')
        imgLeft = pair_img [0:photo_height,0:image_width] #Y+H and X+W
        imgRight = pair_img [0:photo_height,0:photo_width] #Y+H and X+W


        # Implementing calibration data
        print('Read calibration data and rectifying stereo pair...')
    #    print('Current directory:',os.getcwd())
#        os.chdir('./Data')
        print('Current directory:',os.getcwd())
        calibration = StereoCalibration(input_folder='calibData3D')
        rectified_pair = calibration.rectify((imgLeft, imgRight))
        #cv2.imshow('Left CALIBRATED', rectified_pair[0])
        #cv2.imshow('Right CALIBRATED', rectified_pair[1])
        #cv2.waitKey(0)


        # Depth map function
        global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS
        SWS = 5
        PFS = 5
        PFC = 29
        MDS = -25
        NOD = 128
        TTH = 100
        UR = 10
        SR = 15
        SPWS = 100


        disparity = depthMapTuning.stereo_depth_map(rectified_pair)

        # Set up and draw interface
        # Draw left image and depth map
        axcolor = 'lightgoldenrodyellow'
        fig, ax = plt.subplots()
        plt.subplots_adjust(left=0.15, bottom=0.5)
        fig2 = plt.subplot(1,2,1)
        dmObject = plt.imshow(rectified_pair[0], 'gray')

        saveax = plt.axes([0.3, 0.38, 0.15, 0.04]) #stepX stepY width height
        buttons = Button(saveax, 'Save settings', color=axcolor, hovercolor='0.975')

        buttons.on_clicked(depthMapTuning.save_map_settings)
        loadax = plt.axes([0.5, 0.38, 0.15, 0.04]) #stepX stepY width height
        buttonl = Button(loadax, 'Load settings', color=axcolor, hovercolor='0.975')

        buttonl.on_clicked(depthMapTuning.load_map_settings)

        #draw disparity map
        fig3 = plt.subplot(1,2,2)
        #dmObject = fig3.imshow(disparity, aspect='equal', cmap='jet')
        #fig.colorbar(disparity)

        fig3 = plt.imshow(disparity,aspect='equal', cmap='jet')
        fig3 = plt.colorbar()

        # Draw interface for adjusting parameters
        print('Start interface creation (it takes up to 30 seconds)...')

        #slider menu
        SWSaxe = plt.axes([0.15, 0.01, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        PFSaxe = plt.axes([0.15, 0.05, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        PFCaxe = plt.axes([0.15, 0.09, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        MDSaxe = plt.axes([0.15, 0.13, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        NODaxe = plt.axes([0.15, 0.17, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        TTHaxe = plt.axes([0.15, 0.21, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        URaxe = plt.axes([0.15, 0.25, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        SRaxe = plt.axes([0.15, 0.29, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height
        SPWSaxe = plt.axes([0.15, 0.33, 0.7, 0.025], facecolor=axcolor) #stepX stepY width height

        sSWS = Slider(SWSaxe, 'SWS', 5.0, 255.0, valinit=5)
        sPFS = Slider(PFSaxe, 'PFS', 5.0, 255.0, valinit=5)
        sPFC = Slider(PFCaxe, 'PreFiltCap', 5.0, 63.0, valinit=29)
        sMDS = Slider(MDSaxe, 'MinDISP', -100.0, 100.0, valinit=-25)
        sNOD = Slider(NODaxe, 'NumOfDisp', 16.0, 256.0, valinit=128)
        sTTH = Slider(TTHaxe, 'TxtrThrshld', 0.0, 1000.0, valinit=100)
        sUR = Slider(URaxe, 'UnicRatio', 1.0, 20.0, valinit=10)
        sSR = Slider(SRaxe, 'SpcklRng', 0.0, 40.0, valinit=15)
        sSPWS = Slider(SPWSaxe, 'SpklWinSze', 0.0, 300.0, valinit=100)

        # Connect update actions to control elements
        sSWS.on_changed(depthMapTuning.update)
        sPFS.on_changed(depthMapTuning.update)
        sPFC.on_changed(depthMapTuning.update)
        sMDS.on_changed(depthMapTuning.update)
        sNOD.on_changed(depthMapTuning.update)
        sTTH.on_changed(depthMapTuning.update)
        sUR.on_changed(depthMapTuning.update)
        sSR.on_changed(depthMapTuning.update)
        sSPWS.on_changed(depthMapTuning.update)

        print('Show interface to user')
        plt.show()
