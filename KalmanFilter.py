import Utils
import numpy as np
'''
@author Truong Huy Cuong
'''

errorMotion = 0.1
errorSensor = 0.1

A = np.identity(3)
C = 1

def KalmanFilter(dt, theta, lastMean, lastCovariance, u, z):

    B = np.array([[dt*np.cos(theta),0],
                  [dt*np.sin(theta),0],
                  [0,dt]])
    sigmaMotion = np.random.normal(0, errorMotion)
    sigmaSensor = np.random.normal(0, errorSensor)
    R = np.identity(3) * sigmaMotion
    Q = np.identity(3) * sigmaSensor
    #prediction
    mean = np.dot(A,lastMean) + np.dot(B, u)
    covariance = np.dot(np.dot(A,lastCovariance),A.T) + R
    #correction
    mulMat = np.dot( np.dot(C, covariance),C.T)
    inverseMat = np.linalg.inv( mulMat + Q)
    K = np.dot(np.dot(covariance, C.T), inverseMat)
    mean = mean + np.dot(K, (z - np.dot(C,mean)))
    covariance = np.dot((np.identity(3) - np.dot(K,C)), covariance)

    return mean,covariance
