
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen

'''
@author Truong Huy Cuong
'''
class LRobot:
    def __init__(self, startPos,drawColor = Qt.black,doFill=False):
        self.drawColor = drawColor
        self.doFill = doFill
        self.Reset(startPos)

    def Reset(self,startPos):
        # position x y
        self.pos = np.copy(startPos)
        self.prevPos = np.copy(startPos)
        # forward vector
        self.forward = np.array([1, 0])
        # rectangle bounding box
        self.size = 30.0
        self.rsize = np.array([self.size, self.size])
        #sensor configs
        self.sThreshold = 200
        self.updateSensors()
        self.sDistances = np.full(len(self.sensors), self.sThreshold)
        self.isActive = True
        #velocity-based model
        self.v = 0  #velocity
        self.rotation = 0 #omega = rotation speed
        self.theta = 0 #orientation = angle between Ox and forward


    def updateSensors(self):
        xAxes = self.forward
        self.sensors = []
        offset= 30
        for time in range(12):
            angle = np.pi * (offset * time)/180
            temp = np.array([np.cos(angle) * xAxes[0] - np.sin(angle) * xAxes[1],
             np.sin(angle) * xAxes[0] + np.cos(angle) * xAxes[1],
             ])
            self.sensors.append(self.pos + temp*self.rsize[0])

    #call from GUI
    def CalculateTransform(self, dt):

        curLocationMatrix = np.array([self.pos[0], self.pos[1], self.theta])

        rotationMatrix = np.array([[dt * np.cos(self.theta),0],
                                   [dt * np.sin(self.theta),0],
                                   [0,dt]])

        translationMatrix = np.array([self.v, self.rotation])

        retMatrix = curLocationMatrix + np.dot(rotationMatrix , translationMatrix)

        tempPos = np.array([retMatrix[0], retMatrix[1]])
        tempTheta = retMatrix[2]


        return tempPos,tempTheta

    def UpdateTransform(self,newpos,newtheta):
        self.prevPos = np.copy(self.pos)
        self.pos = np.copy(newpos)
        self.theta = newtheta
        xAxes = np.array([1, 0])
        self.forward = np.array(
            [np.cos(self.theta) * xAxes[0] - np.sin(self.theta) * xAxes[1],
             np.sin(self.theta) * xAxes[0] + np.cos(self.theta) * xAxes[1],
             ])

        self.forward /= np.linalg.norm(self.forward)

    #call from GUI
    def Draw(self, qp):
        #body
        origin = np.array([self.pos[0] - self.rsize[0], self.pos[1] - self.rsize[0]])
        if not self.doFill:
            pen = QPen(self.drawColor, 1.5, Qt.SolidLine)
            qp.setPen(pen)
        else:
            qp.setBrush(self.drawColor)
        qp.drawEllipse(origin[0], origin[1], self.rsize[0] * 2, self.rsize[0] * 2)
        # forward
        pen2 = QPen(Qt.red, 2.5, Qt.SolidLine)
        qp.setPen(pen2)
        f = np.copy(self.forward)  # Utils.normalize(self.forward)
        qp.drawLine(self.pos[0], self.pos[1], self.pos[0] + f[0] * self.size, self.pos[1] + f[1] * self.size)

        return True
