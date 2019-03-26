
import numpy as np
from PyQt5.QtCore import Qt, QLineF, QPointF
from PyQt5.QtGui import QPen, QBrush, QPolygonF, QPainterPath
from collections import namedtuple
'''
@author Truong Huy Cuong
'''

Feature = namedtuple("Feature", ["r", "phi", "s","px","py"])

Ellipse = namedtuple("Ellipse",["cx","cy","w","h","a"])

class LRobot:
    MAXIMUM_ELLIPSES = 10
    def __init__(self, startPos,drawColor = Qt.black,doFill=False):
        self.drawColor = drawColor
        self.doFill = doFill
        self.Reset(startPos)

    def Reset(self,startPos):
        # position x y
        self.pos = np.copy(startPos)
        self.prevPos = np.copy(startPos)
        self.prevEPos = np.zeros(3)
        # forward vector
        self.forward = np.array([1, 0])
        # rectangle bounding box
        self.size = 30.0
        self.rsize = np.array([self.size, self.size])
        #sensor configs
        self.sThreshold = 250
        self.isActive = True
        #velocity-based model
        self.v = 0  #velocity
        self.rotation = 0 #omega = rotation speed
        self.theta = 0 #orientation = angle between Ox and forward
        self.prevTheta = 0
        self.features = []
        self.path = QPainterPath()
        self.path.moveTo(QPointF(self.pos[0],self.pos[1]))
        self.pathEstimation = QPainterPath()
        self.pathEstimation.moveTo(QPointF(startPos[0],startPos[1]))
        self.ellipses = []

    def closePaths(self):
        self.path.closeSubpath()
        self.pathEstimation.closeSubpath()
        self.ellipses.clear()

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
        if self.prevPos[0] != newpos[0] or self.prevPos[1] != newpos[1]:
                self.path.lineTo(QPointF(newpos[0],newpos[1]))

        self.prevPos = np.copy(self.pos)
        self.prevTheta = newtheta
        self.pos = np.copy(newpos)
        self.theta = newtheta

        xAxes = np.array([1, 0])
        self.forward = np.array(
            [np.cos(self.theta) * xAxes[0] - np.sin(self.theta) * xAxes[1],
             np.sin(self.theta) * xAxes[0] + np.cos(self.theta) * xAxes[1],
             ])

        self.forward /= np.linalg.norm(self.forward)


    def UpdateSensor(self,beacon):
        distance = beacon.pos - self.pos
        norm = np.linalg.norm(distance)
        isDetected = norm < self.sThreshold
        if isDetected:
            range = norm
            phi = np.arctan2(distance[1], distance[0]) - self.theta
            signature = beacon.id
            temp = np.array([range,phi,signature])
            sigmas = np.array([np.random.normal(0, 0.01),np.random.normal(0, 0.01),signature])
            #sigmas = np.array([0.1,0.1,1])
            #sigmas = np.array([3, 3, 1])
            temp = temp + sigmas
            f = Feature(temp[0],temp[1], temp[2],beacon.pos[0],beacon.pos[1])
            self.features.append(f)

        return isDetected

    def GetMeasurement(self):
        z = np.zeros(3)
        for f in self.features:
            temp = np.array([f.px - f.r * np.cos(f.phi),
                             f.py - f.r * np.sin(f.phi),
                             self.theta ])
            z += temp
            #break;

        n = len(self.features)
        if n > 0:
            z /= n
        else:
            z = np.array([self.prevPos[0],self.prevPos[1],self.prevTheta])
        return z

    def Eigsorted(self,cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:, order]

    def AddEstimatedPath(self, mean, covariance,allowAddEllipse):

        #tempVec = mean - self.prevEPos

        #dist2 = tempVec[0] * tempVec[0] + tempVec[1] * tempVec[1]

        if self.prevEPos[0] != mean[0] or self.prevEPos[1] != mean[1]:
        #if dist2 > 50:
                self.pathEstimation.lineTo(QPointF(mean[0],mean[1]))
                if allowAddEllipse:
                    # eclipse
                    [w, v] = self.Eigsorted(covariance)
                    width, height, d = 2 * 1 * np.sqrt(w)
                    #theta = np.degrees(np.arctan2(*v[:, 0][::-1]))
                    theta = np.degrees(mean[2])
                    #print("theta="+str(theta))
                    pos = np.copy(mean)
                    e = Ellipse(pos[0],pos[1],width,height,theta)
                    self.ellipses.append(e)
                    if len(self.ellipses) > self.MAXIMUM_ELLIPSES:
                        self.ellipses.remove(self.ellipses[0])

        self.prevEPos = np.copy(mean)


    #call from GUI
    def Draw(self, qp):

        if not self.doFill:
            #sensor range
            pen = QPen(Qt.yellow, 1.5, Qt.SolidLine)
            qp.setPen(pen)
            o2 = np.array([self.pos[0] - self.sThreshold, self.pos[1] - self.sThreshold])
            qp.drawEllipse(o2[0], o2[1], self.sThreshold*2, self.sThreshold*2)

            pen3 = QPen(Qt.green, 2, Qt.SolidLine)
            qp.setPen(pen3)
            for feature in self.features:
                qp.drawLine(self.pos[0],self.pos[1],feature.px,feature.py)
            #local pos trace
            pen = QPen(Qt.black, 2, Qt.SolidLine)
            qp.setPen(pen)
            qp.drawPath(self.path)
            #estimated pos trace
            pen = QPen(Qt.darkGreen, 2, Qt.DotLine)
            qp.setPen(pen)
            qp.drawPath(self.pathEstimation)

            pen = QPen(Qt.darkRed, 2, Qt.SolidLine)
            qp.setPen(pen)
            qp.setBrush(Qt.darkRed)
            for ellipse in self.ellipses:
                tempOrigin = np.array([ellipse.cx ,ellipse.cy ])

                tempAngle = ellipse.a.astype(dtype=float)

                tempOrigin = QPointF(tempOrigin[0],tempOrigin[1])

                #for j in range(20):
                #    point = np.random.multivariate_normal(ellipse.mean,ellipse.cov)
                #    qp.drawPoint(QPointF(point[0],point[1]))
                qp.translate(tempOrigin)
                qp.rotate(tempAngle)
                qp.drawEllipse(QPointF(0,0),ellipse.w,ellipse.h)
                qp.rotate(-tempAngle)
                qp.translate(-tempOrigin)


        qp.setBrush(Qt.transparent)
        # body
        origin = np.array([self.pos[0] - self.rsize[0], self.pos[1] - self.rsize[0]])
        pen = QPen(self.drawColor, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        if self.doFill:
            qp.setBrush(self.drawColor)
        else:
            qp.setBrush(Qt.transparent)

        qp.drawEllipse(origin[0], origin[1], self.rsize[0] * 2, self.rsize[0] * 2)
        # forward
        pen2 = QPen(Qt.red, 2.5, Qt.SolidLine)
        qp.setPen(pen2)
        f = np.copy(self.forward)  # Utils.normalize(self.forward)
        qp.drawLine(self.pos[0], self.pos[1], self.pos[0] + f[0] * self.size, self.pos[1] + f[1] * self.size)

        return True
