
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

    def __init__(self,parent=None):
        super(Robot, self).__init__()
        #print("myId: "+str(self.id))
        self.setPosition(500,500)
        self.forward = np.array([1.,0.])
        self.vl = 0#np.array([0.,0.])
        self.vr = 0#np.array([0.,0.])
        self.v = (self.vl+self.vr)/2
        self.size = 100.0
        self.l = 500/2
        self.ICC = self.pos
        xAxes =np.array([1,0])
        self.theta = np.arccos(np.dot(self.forward, xAxes) / (
                    np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))  # self.angle(self.forward,np.array([1,0]))

    #angle between Ox and forward vector

    def setPosition(self,x,y):
        self.pos = np.array([x,y])



    def updateTransform(self,dt):
        delta = self.vr - self.vl
        if delta != 0:
            xAxes =np.array([1,0])
            self.rotation = delta / self.l
            self.R = self.l/2 * (self.vr + self.vl)/delta
            temp1 = self.vl * xAxes#self.forward
            temp2 = self.vr * xAxes#self.forward

            self.forward = (temp1 + temp2)/2

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
            
            print("forward= "+str(self.forward))
            #self.pos = self.pos + self.forward

        elif self.vr == -self.vl and self.vr !=0:
            self.forward = np.zeros((1,2))
            self.rotation = 2 /self.l * self.vr
        else:
            self.pos = self.pos + self.forward

        return True

    def updatePosition(self,dt):
        delta = self.vr - self.vl#np.linalg.norm( self.vr - self.vl)

        if delta !=0 :
            #print("inside")
            xAxes =np.array([1,0])
            temp =  np.arccos(np.dot(self.forward, xAxes) / (np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))# self.angle(self.forward,np.array([1,0]))


            self.rotation = delta / self.l
            #self.R = self.l / 2 * np.linalg.norm(self.vl + self.vr) / np.linalg.norm(self.vr - self.vl)
            self.R = self.l * (self.vl + self.vr) / (2*delta)

            print("theta= "+str(temp))
            print("rotation= "+str(self.rotation))
            print("R= "+str(self.R))
            self.ICC = np.array([self.pos[0] - self.R *np.sin(temp), self.pos[1] + self.R *np.cos(temp)])
            newPos,theta= self.rotate(dt)

            self.pos = newPos

            #self.forward = ( (self.vr + self.vl) / 2)
            #self.forward = self.forward/np.linalg.norm(self.forward)
            #self.forward *= np.array([xAxes[0] - np.sin(theta), xAxes[1] + self.R *np.cos(theta)])
            #self.forward = Utils.normalize(self.forward)
            #self.forward[0] =  xAxes[0] * np.cos(theta) - xAxes[1] * np.sin(theta)
            #self.forward[1] = xAxes[0] * np.sin(theta) + xAxes[1] * np.cos(theta)
            #np.cos(theta) = np.dot(self.forward, xAxes)
            print("f = "+str( self.forward)+",new theta="+str(theta))
            print(self.pos)
        else:
            print("outside")
            self.pos = self.pos + self.forward




    def rotate(self,dt):
        xAxes =np.array([1,0])
        theta = np.arccos(np.dot(self.forward, xAxes) / (np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))# self.angle(self.forward,np.array([1,0]))
        odt = self.rotation * dt
        rotationMat = np.array([[np.cos(odt),-np.sin(odt),0]
                               ,[np.sin(odt),np.cos(odt),0]
                               ,[0,0,1]])
        m2 = np.array([ self.pos[0] -  self.ICC[0],  self.pos[1] -  self.ICC[1],  theta])
        m3 = np.array([ self.ICC[0], self.ICC[1], odt])

        ret = np.dot(rotationMat,m2)+m3
        newPos = np.array([ret[0],ret[1]])
        theta = ret[2]
        return newPos,theta


    def setDistanceBetweenWheelAndOrigin(self,value):
        if(value >0 and value < self.radius):
            self.l = value


    def draw(self,qp):
        #body
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawEllipse(self.pos[0], self.pos[1], self.size, self.size)
        #forward
        origin = self.pos + self.size/2
        pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        qp.setPen(pen2)
        qp.drawLine(origin[0], origin[1], origin[0]+self.forward[0]*50, origin[1]+self.forward[1]*50)

        #ICC debug
        pen3 = QPen(Qt.blue, 1.5, Qt.SolidLine)
        qp.setPen(pen3)
        qp.drawEllipse(self.ICC[0], self.ICC[1], self.size/3, self.size/3)


        #left wheel - todo come back later
        '''
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