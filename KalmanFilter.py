import Utils
import numpy as np
import math
'''
@author Truong Huy Cuong
'''

#sdX

A = np.identity(3)
C = np.identity(3)

def KalmanFilter(dt, theta, lastMean, lastCovariance, u, z):

    B = np.array([[dt*np.cos(theta),0],
                  [dt*np.sin(theta),0],
                  [0,dt]])

    sigmaMotion = np.array([250,10,20])#np.array([0.1,0.3,0])
    sigmaSensor = np.array([100,50,0.1])#np.array([0.2,0.3,0.1])
    R = np.identity(3) * sigmaMotion
    Q = np.identity(3) * sigmaSensor

    '''
    #prediction
    pMean = A @ lastMean + B @ u
    pCovariance = A @ lastCovariance @ A.T + R
    #correction
    mulMat = C @ pCovariance @ C.T + Q
    invMat = np.linalg.inv(mulMat)
    K = pCovariance @ C @ invMat
    mean = pMean + K @ (z - C @ pMean)
    covariance = (np.identity(3) - K @ C ) @ pCovariance
    '''

    #prediction
    pMean = np.dot(A,lastMean) + np.dot(B, u)
    pCovariance = np.dot(np.dot(A,lastCovariance),A.T) + R
    #correction
    mulMat = np.dot( np.dot(C, pCovariance),C.T)
    inverseMat = np.linalg.inv( mulMat + Q)
    K = np.dot(np.dot(pCovariance, C.T), inverseMat)
    mean = pMean + np.dot(K, (z - np.dot(C,pMean)))
    covariance = np.dot((np.identity(3) - np.dot(K,C)), pCovariance)


    if math.isnan(mean[0]):
        print("im here")

    return pMean,pCovariance,mean,covariance
