
from PyQt5.QtWidgets import QMainWindow, QFrame, QDesktopWidget, QApplication
from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog,
QDialogButtonBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
QLabel, QLineEdit, QMenu, QMenuBar, QPushButton, QSpinBox, QTextEdit,
QVBoxLayout)
from PyQt5.QtCore import Qt, QBasicTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import Utils
import numpy as np
from Object import Object
class Robot(Object):
    def angle(v1, v2):
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        return angle

    def getVelocity(self):

        return self.forward * (self.vl + self.vr)/2

    def getShape(self):
        return 2#Object.Shape.CIRCLE

    def __init__(self,parent=None,assignId=True):
        super(Robot, self).__init__(assignId=assignId)
        #print("myId: "+str(self.id))
        self.point = np.array([0,0])
        self.normal = np.array([0,0])

        self.setPosition(500,500)
        self.forward = np.array([1.,0.])
        self.vl = 0#np.array([0.,0.])
        self.vr = 0#np.array([0.,0.])
        self.v = (self.vl+self.vr)/2
        self.size = 100.0
        self.rsize = np.array([self.size,self.size])
        self.l = 250
        self.ICC = self.pos
        self.rotation =0
        self.R = 0
        xAxes =np.array([1,0])
        self.theta = np.arccos(np.dot(self.forward, xAxes) / (
                    np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))  # self.angle(self.forward,np.array([1,0]))

        self.sThreshold = 100
        self.updateSensors()
        self.sDistances = np.zeros((len(self.sensors),))

    #angle between Ox and forward vector

    def updateSensors(self):
        xAxes = self.forward

        self.sensors = []
        offset= 30
        for time in range(12):
            angle = np.pi * (offset * time)/180
            temp = np.array([np.cos(angle) * xAxes[0] - np.sin(angle) * xAxes[1],
             np.sin(angle) * xAxes[0] + np.cos(angle) * xAxes[1],
             ])
            self.sensors.append(self.pos + temp*self.rsize[0]/2)




    def setPosition(self,x,y):
        self.pos = np.array([x,y])

    def checkLinePoint(self,ps1,pe1,ps2,pe2):
        A1 = pe1[1] - ps1[1]
        B1 = ps1[0] - pe1[0]
        A2 = pe2[1] - ps2[1]
        B2 = ps2[0] - pe2[0]
        delta = A1 * B2 - A2 * B1
        if delta == 0: #parallel
            return None
        C2 = A2 * ps2[0] + B2 * ps2[1]
        C1 = A1 * ps1[0] + B1 * ps1[1]
        invdelta = 1. / delta

        ret = [ (B2*C1 - B1*C2)*invdelta, (A1*C2 - A2*C1)*invdelta ]



        return ret





    def checkLineRect(self,p1,p2,rx,ry,rw,rh):
        r1= [rx - rw/2,ry-rh/2]
        r2= [rx + rw/2,ry-rh/2]
        r3= [rx - rw/2,ry+rh/2]
        r4= [rx + rw/2,ry+rh/2]

        min_x = np.min([r1[0], r2[0], r3[0], r4[0]])
        min_y = np.min([r1[1], r2[1], r3[1], r4[1]])
        max_x = np.max([r1[0], r2[0], r3[0], r4[0]])
        max_y = np.max([r1[1], r2[1], r3[1], r4[1]])

        if ((p1[0] < min_x and p2[0] < min_x) or
            (p1[1] > max_y and p2[1] > max_y) or
            (p1[0] > max_x and p2[0] > max_x) or
            (p1[1] < min_y and p2[1] < min_y)
        ):
            return None
        if p2[0] < min_x:
            if p2[1] > min_y and p2[1] < max_y:
                if p1[0] > max_x:
                    return self.checkLinePoint(p1, p2, [max_x, min_y], [max_x, max_y])
                elif p1[1] > max_y:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [max_x, min_y])

            elif p2[1] < min_y:
                if p1[0]>max_x:
                    return self.checkLinePoint(p1, p2, [max_x, min_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])

            else:
                if p1[0]>max_x:
                    return self.checkLinePoint(p1, p2, [max_x, min_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [max_x, min_y])
        elif p2[0] > max_x:
            if p2[1] > min_y and p2[1] < max_y:
                if p1[0] < min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                elif p1[1] > max_y:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [max_x, min_y])
            elif p2[1] < min_y:
                if p1[0]< min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])
            else:
                if p1[0]< min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [max_x, min_y])
        else:
            if p2[1] < min_y:
                if p1[0] < min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                elif p1[0]>max_x:
                    return self.checkLinePoint(p1, p2, [max_x, max_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])
            if (p2[1] > max_y):
                if p1[0] < min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                elif p1[0]>max_x:
                    return self.checkLinePoint(p1, p2, [max_x, min_y], [max_x, max_y])
                else:
                    return self.checkLinePoint(p1, p2, [min_x, min_x], [max_x, min_y])
            else:
                if p1[0]<min_x:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [min_x, max_y])
                elif p1[0]>max_x:
                    return self.checkLinePoint(p1, p2, [max_x, min_y], [max_x, max_y])
                if p1[1]<min_y:
                    return self.checkLinePoint(p1, p2, [min_x, min_y], [max_x, min_y])
                elif p1[1]>max_y:
                    return self.checkLinePoint(p1, p2, [min_x, max_y], [max_x, max_y])


        return None


    def checkCollision(self, other, doResponse=True):
        '''
        for i,sensor in enumerate(self.sensors) :
            norVec = Utils.normalizeByDivideNorm(sensor-self.pos)

            contractPoint = self.checkLineRect(sensor, sensor+norVec*self.sThreshold, other.pos[0], other.pos[1],
                                                                  other.rsize[0], other.rsize[1])

            if contractPoint != None:
                if not ( contractPoint[0] < other.pos[0]-other.rsize[0]/2 or   contractPoint[0] > other.pos[0]+other.rsize[0]/2 or
                        contractPoint[1] < other.pos[1]-other.rsize[1] / 2 or contractPoint[1] > other.pos[1]+other.rsize[1] / 2
                ):
                    self.sDistances[i] = np.linalg.norm(contractPoint-sensor)# np.array([contractPoint])


        '''
        return super(Robot,self).checkCollision(other,doResponse)


    def onCollisionWith(self, obj, normalX, normalY, contractPoint):

        projection = np.array([0,0])
        if normalX !=0 and normalY !=0:
            vec = np.array([0,0])
            self.vl = self.vr = 0
            return True
        else:
            if normalX !=0:
                if normalX > 0:
                    vec = np.array([0,-1])
                else:
                    vec = np.array([0,1])
            else:
                if normalY > 0:
                    vec = np.array([-1,0])
                else:
                    vec = np.array([1,0])

        planeVec = vec
        projection = np.multiply(self.forward, planeVec) / (planeVec[0] ** 2 + planeVec[1] ** 2) * planeVec


        if projection[0] !=0:
             if projection[0] >0:
                 vec = np.array([ 1,0 ])
             else:
                 vec = np.array([ -1,0 ])
        else:
             if projection[1] >0:
                 vec = np.array([ 0,1 ])
             else:
                 vec = np.array([ 0,-1 ])

        n =  Utils.normalizeByDivideNorm(contractPoint - self.pos)


        self.forward = Utils.normalizeByDivideNorm(vec)
        lastTheta= np.rad2deg(self.theta)
        self.theta = np.arccos(np.dot(self.forward, np.array([1, 0])) / (
                np.linalg.norm(self.forward)*np.linalg.norm(np.array([1, 0]))))  # self.angle(self.forward,np.array([1,0]))

        temTheta = np.rad2deg(self.theta)

        if lastTheta <0:
             self.theta = -self.theta


        return True


    def updateTransform(self,dt):
        xAxes = np.array([1, 0])
        delta = self.vr - self.vl
        if delta != 0:

            self.rotation = delta / self.l
            self.R = self.l/2 * (self.vr + self.vl)/delta
            #temp1 = self.vl * xAxes#self.forward
            #temp2 = self.vr * xAxes#self.forward

            #self.forward = (temp1 + temp2)/2

            self.ICC = np.array([self.pos[0]- self.R * np.sin(self.theta),
                                 self.pos[1]+ self.R * np.cos(self.theta)])

            odt = self.rotation * dt
            rotMatrix = np.array([[np.cos(odt),-np.sin(odt),0],
                                  [np.sin(odt),np.cos(odt),0],
                                  [0,0,1]])
            toOriginMatrix = np.array([self.pos[0]-self.ICC[0],
                                       self.pos[1]-self.ICC[1],
                                       self.theta])
            backLocationMatrix = np.array([self.ICC[0],
                                           self.ICC[1],
                                           odt])
            retMatrix = np.dot(rotMatrix,toOriginMatrix) + backLocationMatrix

            self.pos = np.array([retMatrix[0],retMatrix[1]])
            self.theta = retMatrix[2]

            self.forward = np.array(
                [np.cos(self.theta)*xAxes[0] - np.sin(self.theta)*xAxes[1],
                    np.sin(self.theta) * xAxes[0] + np.cos(self.theta) * xAxes[1],
                ])
            #aVec = self.ICC - self.pos
            #aVec= np.linalg.norm(aVec,ord=1,keepdims=True)
            #self.forward = np.array([aVec[1],-aVec[0]])
            #temp = np.arccos(np.dot(self.forward, xAxes) / (
            #        np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))  # self.angle(self.forward,np.array([1,0]))

            #print("forward= " + str(self.forward))
            #self.pos = self.pos + self.forward

        elif self.vr == -self.vl and self.vr !=0:
            #self.forward = np.zeros((1,2))
            self.rotation = 2 /self.l * self.vr
        elif self.vr !=0 :
            #if self.vr < 0:
            #    self.theta = -self.theta
            self.forward = np.array(
                [np.cos(self.theta) * xAxes[0] - np.sin(self.theta) * xAxes[1],
                 np.sin(self.theta) * xAxes[0] + np.cos(self.theta) * xAxes[1],
                 ])
            #print("forward= " + str(self.forward))
            f = np.copy(self.forward)
            f/= np.linalg.norm(self.forward)

            self.rotation =0
            self.ICC = self.pos
            self.pos = self.pos + f * (self.vl+self.vr)/2


        self.updateSensors()
        return True

    def clone(self,assignId=False ):
        cloner = Robot(assignId)
        cloner.pos = np.copy(self.pos)
        cloner.size = np.copy(self.size)
        cloner.rsize = np.copy(self.rsize)
        cloner.ICC   = np.copy(self.ICC)
        cloner.forward = np.copy(self.forward)
        cloner.rotation = self.rotation
        cloner.theta = self.theta
        cloner.vl = self.vl
        cloner.vr = self.vr
        cloner.R = self.R
        cloner.l = self.l
        return cloner

    def setDistanceBetweenWheelAndOrigin(self,value):
        if(value >0 and value < self.radius):
            self.l = value


    def draw(self,qp):
        #body
        origin = np.array([self.pos[0] - self.rsize[0]/2,self.pos[1] - self.rsize[0]/2])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawEllipse(origin[0], origin[1], self.rsize[0], self.rsize[0])

        #qp.drawPie(QRectF(origin[0], origin[1], self.size, self.size), 0, 5760)
        #forward
        pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        qp.setPen(pen2)
        f = self.forward #Utils.normalize(self.forward)
        qp.drawLine(self.pos[0], self.pos[1], self.pos[0]+f[0]*self.size/2, self.pos[1]+f[1]*self.size/2)

        #ICC debug
        pen3 = QPen(Qt.magenta, 1.5, Qt.SolidLine)
        qp.setPen(pen3)
        qp.drawEllipse(self.ICC[0], self.ICC[1], self.size/4, self.size/4)


        #bounding box debug
        #pen4 =QPen(Qt.yellow, 1.5, Qt.SolidLine)
        #qp.setPen(pen4)
        #qp.drawRect(origin[0], origin[1], self.rsize[0], self.rsize[1])

        #velocity debug
        #vec=self.getVelocity()
        #pen5 = QPen(Qt.magenta, 1.5, Qt.SolidLine)
        #qp.setPen(pen5)
        #qp.drawLine(self.pos[0], self.pos[1], self.pos[0] +vec[0], self.pos[1] +vec[1])
        #left wheel - todo come back later

        pen6 = QPen(Qt.green, 5, Qt.SolidLine)
        qp.setPen(pen6)
        qp.drawPoint(self.point[0],self.point[1])


        #pen = QPen(Qt.black, 10, Qt.SolidLine)
        #qp.setPen(pen)
        #qp.drawLine(self.point[0],self.point[1],self.point[0]+self.normal[0],self.point[1]+self.normal[1])

        pen = QPen(Qt.black, 10, Qt.SolidLine)
        qp.setPen(pen)

        #draw texts

        pVec = np.array([self.forward[1],-self.forward[0]])
        lefTextPos = self.pos + pVec * 50

        pen = QPen(Qt.blue, 1, Qt.SolidLine)
        qp.setPen(pen)

        #qp.drawPoint(textPos[0], textPos[1])
        qp.drawText(QPointF(lefTextPos[0] ,lefTextPos[1] ),str(self.vl))

        rightTextPos = self.pos + pVec * -50
        pen = QPen(Qt.red, 1, Qt.SolidLine)
        qp.setPen(pen)

        # qp.drawPoint(textPos[0], textPos[1])
        qp.drawText(QPointF(rightTextPos[0], rightTextPos[1]), str(self.vr))



        for i,sensor in enumerate(self.sensors) :
            pen6 = QPen(Qt.darkCyan, 0.01, Qt.SolidLine)
            qp.setPen(pen6)
            norVec = Utils.normalizeByDivideNorm(sensor - self.pos) *self.sThreshold
            qp.drawText(QPointF(sensor[0] +norVec[0], sensor[1]+norVec[1]), str("{:.1f}".format(self.sDistances[i])))

            #pen6 = QPen(Qt.darkCyan, 6, Qt.SolidLine)
            #qp.setPen(pen6)
            #qp.drawLine(sensor[0], sensor[1], sensor[0] + norVec[0], sensor[1] + norVec[1])

            #qp.drawLine(sensor[0], sensor[1], self.sDistances[i][0],self.sDistances[i][1])

        '''
        pen6 = QPen(Qt.darkBlue, 1.5, Qt.SolidLine)
        qp.setPen(pen6)
        qp.drawLine(self.pos[0], self.pos[1], self.point[0],self.point[1])

        pen6 = QPen(Qt.darkBlue, 1.5, Qt.SolidLine)
        qp.setPen(pen6)
        qp.drawLine(self.pos[0], self.pos[1], self.point[0],self.point[1])
        
        pen3 = QPen(Qt.blue, 0.5, Qt.SolidLine)
        qp.setPen(pen3)
        qp.drawLine(55, 50, 75, 50)
        #right wheel
        pen4 = QPen(Qt.black, 0.5, Qt.SolidLine)
        qp.setPen(pen4)
        qp.drawLine(55, 80, 75, 80)
        '''
        return True

    '''
    def paintEvent(self,event):
        print("updating render")
        painter = QPainter(self)
        painter.setBrush(Qt.blue)
        # body
        #pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        #painter.setPen(pen)
        painter.drawPie(QRectF(self.pos[0], self.pos[1], self.size, self.size), 0, 5760)
        #painter.drawEllipse(self.pos[0], self.pos[1], self.size, self.size)
        # forward
        origin = self.pos + self.size / 2
        #pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        #painter.setPen(pen2)
        painter.drawLine(origin[0], origin[1], origin[0] + self.forward[0], origin[1] + self.forward[1])

        # ICC debug
        #pen3 = QPen(Qt.blue, 1.5, Qt.SolidLine)
        #painter.setPen(pen3)
        painter.drawEllipse(self.ICC[0], self.ICC[1], self.size / 3, self.size / 3)



        # painter.translate(0, self.rect().height())
        #print(str(self.x) + "," + str(self.y))
        #print("shape = "+str(self.pos.shape))

        #origin = np.array([self.pos[0]-self.size/2, self.pos[1]-self.size/2])

        #painter.drawPie(QtCore.QRectF(origin[0], origin[1],self.size,self.size),0, 5760)
        #painter.drawPie(QtCore.QRectF(self.pos[0], self.pos[1],self.size,self.size),0, 5760)
        #painter.setBrush(QtCore.Qt.red)
        #painter.drawLine(QtCore.QLineF(self.pos[0],self.pos[1],self.pos[0]+self.forward[0]*100,self.pos[1]+self.forward[1]*100))


        #painter.drawPie(QtCore.QRectF(self.ICC[0], self.ICC[1], self.size/3, self.size/3), 0, 5760)

        #painter.(QtCore.QPointF(self.ICC[0],self.ICC[1]))
        # painter.rotate(-self.currentAngle)
        # painter.drawRect(QtCore.QRect(33, -4, 15, 8))
    '''