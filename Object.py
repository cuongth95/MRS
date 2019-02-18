import abc
import math
from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
class Object:#(QtWidgets.QWidget):
    #static things
    idCounter=0
    @staticmethod
    def getId(self):
        ret = self.idCounter + 1
        self.idCounter+=1
        return ret



    #constructor
    def __init__(self,parent=None,assignId=True):
        #QtWidgets.QWidget.__init__(self, parent)
        if assignId:
            self.id = self.getId(self)
        else:
            self.id = -1
        self.pos = np.array([0,0])
        self.forward = np.array([1,0]) #right
        self.rsize = np.array([0,0])

    def setX(self,value):
        self.pos[0] = value

    def setY(self,value):
        self.pos[1] = value

    @abc.abstractmethod
    def draw(self,qp):
        return True

    @abc.abstractmethod
    def getShape(self):
        return 1#shape.SQUARE
    @abc.abstractmethod
    def getVelocity(self):
        return self.forward

    @abc.abstractmethod
    def onCollisionWith(self,obj,normalX,normalY,colTime):
        return True

    def AABB(self,obj):
        b1 = self.pos
        b1Width = self.rsize[0]
        b1Height = self.rsize[1]

        myVeloc= self.getVelocity()
        obVeloc= obj.getVelocity()

        b1vx = myVeloc[0] - obVeloc[0]
        b1vy = myVeloc[1] - obVeloc[1]

        b2 = obj.pos
        b2Width = obj.rsize[0]
        b2Height = obj.rsize[1]

        if b1vx > 0.0:
            xInvEntry = b2[0] - b1[0] + b1Width
            xInvExit = b2[0] + b2Width - b1[0]
        else:
            xInvEntry = b2[0] + b2Width - b1[0]
            xInvExit = b2[0] - (b1[0]+b1Width)

        if b1vy > 0.0:
            yInvEntry = b2[1] - (b1[1]+b1Height)
            yInvExit = b2[1]+b2Height - b1[1]
        else:
            yInvEntry = b2[1] + b2Height - b1[1]
            yInvExit = b2[1] - (b1[1]+b1Height)

        if b1vx == 0.0:
            xEntry = -math.inf
            xExit = math.inf
        else:
            xEntry = xInvEntry/b1vx
            xExit = xInvExit/b1vx

        if b1vy == 0.0:
            yEntry = -math.inf
            yExit = math.inf
        else:
            yEntry = yInvEntry/b1vy
            yExit = yInvExit/b1vy

        if xEntry > yEntry:
            entryTime = xEntry
        else:
            entryTime = yEntry

        if xExit < yExit:
            exitTime = xExit
        else:
            exitTime = yExit

        if entryTime > exitTime or xEntry < 0.0 and yEntry < 0.0 or xEntry > 1.0 or yEntry > 1.0:
            normalx = 0.0
            normaly = 0.0
            return 1.0,normalx,normaly
        else:
            if xEntry > yEntry:
                if xInvEntry <0:
                    normalx = 1.0
                    normaly = 0.0
                else:
                    normalx = -1.0
                    normaly = 0.0
            else:
                if yInvEntry <0:
                    normalx = 0.0
                    normaly = 1.0
                else:
                    normalx = 0.0
                    normaly = -1.0
            return entryTime,normalx,normaly

    def circleRect2(self, cx, cy, rad, rx, ry, rw, rh):
        px = cx
        py = cy

        rLeft = rx - rw/2
        rRight= rx + rw/2
        rTop = ry - rh/2
        rBot = ry + rh/2

        if px<rLeft:
            px = rLeft
        elif px > rRight:
            px = rRight
        if py < rTop:
            py =rTop
        elif py> rBot:
            py = rBot
        self.point = np.array([px,py])
        dx = cx - px
        dy = cy - py

        d = np.array([dx,dy])
        temp = math.sqrt(dx**2 + dy**2)
        #temp = dx**2 + dy**2

        #dRad = rad**2

        if temp <= rad/2:
            return True
        else:
            return False

    def circleRect(self,cx,cy,rad,rx,ry,rw,rh):
        testX =cx
        testY =cy
        if (cx > rx):
            testX = rx
        elif (cx < rx+rw):
            testX = rx + rw
        if (cy > ry):
            testY = ry
        elif (cy < ry+rh):
            testY = ry+rh

        distX = cx-testX
        distY = cy-testY
        dist = math.sqrt(distX*distX + distY*distY)

        if dist <= rad:
            return True
        else:
            return False

    def checkCollision(self,other):
        isCollision = False

        selfShape = self.getShape()
        otherShape =other.getShape()
        '''
        if selfShape == 2 and otherShape == 1:
            deltaVec=  self.pos - other.pos
            isCollision = self.circleRect(self.pos[0],self.pos[1],self.rsize[0]
                                     ,other.pos[0],other.pos[1],other.rsize[0],other.rsize[1])

            if isCollision:
                self.onCollisionWith(other, 0, 0, 0)
                other.onCollisionWith(self, 0, 0, 0)

                print("collided")
        '''
        #myBox = self.getBoundingBox(self)
        if selfShape == 2 and otherShape == 1:
            if self.quickCheck2(self,other): #self.circleRect(self.pos[0],self.pos[1],self.rsize[0]
                                         #,other.pos[0]-other.rsize[0]/2,other.pos[1]-other.rsize[1] / 2,other.rsize[0],other.rsize[1]):
                print("collided")
                '''
                collideTime,normalX,normalY = self.AABB(other)
                #friction here- but i dont use
    
                if collideTime <= 1:
                    self.onCollisionWith(other,normalX,normalY,collideTime)
                    other.onCollisionWith(self, normalX, normalY, collideTime)
                    isCollision =True
                    print("collided")
                '''

        return isCollision

    def getBoundingBox(self, obj):
        box = Object(assignId=False)
        box.pos = np.copy(obj.pos)
        box.rsize = np.copy(obj.rsize)
        return box

    def quickCheck2(self, a, b):
        return self.circleRect2(a.pos[0], a.pos[1], a.rsize[0], b.pos[0], b.pos[1], b.rsize[0], b.rsize[1])

    def quickCheck(self, a, b):
        return not(a.pos[0] + a.rsize[0] < b.pos[0]
            or a.pos[0] > b.pos[0] + b.rsize[0]
            or a.pos[1] + a.rsize[1] < b.pos[1]
            or a.pos[1] > b.pos[1] + b.rsize[1] )

