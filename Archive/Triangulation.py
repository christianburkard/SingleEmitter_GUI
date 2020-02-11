"""
Created on 11.02.2020

Author: Christian Burkard
Masterthesis: Closed-Loop Control of an Acoustic Levitation System

"""



#C1 and C2 are the cameras matrix (left and right)
#R_0 and T_0 are the transformation between cameras
#Coord1 and Coord2 are the correspondant coordinates of left and right respectively
P1 = np.dot(C1,np.hstack((np.identity(3),np.zeros((3,1)))))

P2 =np.dot(C2,np.hstack(((R_0),T_0)))

for i in range(Coord1.shape[0])
    z = cv2.triangulatePoints(P1, P2, Coord1[i,],Coord2[i,])

    U, S, Vt = linalg.svd(F)
V = Vt.T

#Right epipol
U[:,2]/U[2,2]

# The expected X-direction with C1 camera matri and C1[0,0] the focal length
vecteurX = np.array([(U[:,2]/U[2,2])[0],(U[:,2]/U[2,2])[1],C1[0,0]])
vecteurX_unit = vecteurX/np.sqrt(vecteurX[0]**2 + vecteurX[1]**2 + vecteurX[2]**2)


# The expected Y axis :
height = 2048
vecteurY = np.array([0, height -1, 0])
vecteurY_unit = vecteurY/np.sqrt(vecteurY[0]**2 + vecteurY[1]**2 + vecteurY[2]**2)


# The expected Z direction :
vecteurZ = np.cross(vecteurX,vecteurY)
vecteurZ_unit = vecteurZ/np.sqrt(vecteurZ[0]**2 + vecteurZ[1]**2 + vecteurZ[2]**2)

#Normal of the Z optical (the current Z direction)
Zopitcal = np.array([0,0,1])

cos_theta = np.arccos(np.dot(vecteurZ_unit, Zopitcal)/np.sqrt(vecteurZ_unit[0]**2 + vecteurZ_unit[1]**2 + vecteurZ_unit[2]**2)*np.sqrt(Zopitcal[0]**2 + Zopitcal[1]**2 + Zopitcal[2]**2))

sin_theta = (np.cross(vecteurZ_unit, Zopitcal))[1]

#Definition of the Rodrigues vector and use of cv2.Rodrigues to get rotation matrix
v1 = Zopitcal
v2 = vecteurZ_unit

v_rodrigues = v1*cos_theta + (np.cross(v2,v1))*sin_theta + v2*(np.cross(v2,v1))*(1. - cos_theta)
R = cv2.Rodrigues(v_rodrigues)[0]