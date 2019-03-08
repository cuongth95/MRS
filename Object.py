import abc
import math
from collections import namedtuple
import numpy as np
import Utils
'''
@author Truong Huy Cuong
'''
class Object:#(QtWidgets.QWidget):
    #static things
    idCounter=0
    @staticmethod
    def getId(self):
        ret = self.idCounter + 1
        self.idCounter+=1
        return ret



    #constructor
    def __init__(self,parent=None,assignId=True,collisionShape=1):
        #QtWidgets.QWidget.__init__(self, parent)
        if assignId:
            self.id = self.getId(self)
        else:
            self.id = -1
        self.shape = collisionShape
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
    def draw2(self):
        return True

    @abc.abstractmethod
    def getShape(self):
        return self.shape#shape.SQUARE by default
    @abc.abstractmethod
    def getVelocity(self):
        return self.forward

    @abc.abstractmethod
    def onCollisionWith(self,obj,normalX,normalY,contractPoint):
        return True

    @abc.abstractmethod
    def onCollisionWith2(self, obj):
        return True


    def interactCirclePoint(self,x1,y1,cx,cy,r):
        dx = cx - x1
        dy = cy - y1

        d = np.array([dx,dy])
        magnitude = np.linalg.norm(d)
        normD = d/magnitude

        ret1 = [cx + normD[0] * r,cy + normD[1] * r]
        ret2 = [cx - normD[0] * r,cy - normD[1] * r]


        return ret1,ret2



    def projectOnto(self,a,b):
        dp = np.dot(a,b)
        proj = np.zeros(2)
        proj[0] = (dp / (b[0]**2 + b[1]**2)) * b[0]
        proj[1] = (dp / (b[0]**2 + b[1]**2)) * b[1]

        return proj


    def onRect(self,ret,rtl,rbr):
        return  not(ret[0] <rtl[0] or ret[0] > rbr[0] or
                ret[1] < rtl[1] or ret[1] > rbr[1]
        )






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

    def AABB2(self,a,b,vax,vay):
        b1 = [a.x,a.y]
        b1Width = a.w
        b1Height =a.h

        myVeloc= [vax,vay]
        obVeloc= [0,0]

        b1vx = myVeloc[0] - obVeloc[0]
        b1vy = myVeloc[1] - obVeloc[1]

        b2 = [b.x,b.y]
        b2Width = b.w
        b2Height = b.h

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

    @abc.abstractmethod
    def checkCollision(self,other,doResponse=True):
        isCollision,normal,contractPoint = self.quickCheck2(self, other)
        if isCollision and doResponse:
            self.onCollisionWith(other, normal[0], normal[1], contractPoint)
            other.onCollisionWith(self, -normal[0],-normal[1] , contractPoint)


        return isCollision

    Box = namedtuple('Box','x y w h')
    @abc.abstractmethod
    def getBoundingBox(self):
        box = self.Box(self.pos[0],self.pos[1],self.rsize[0],self.rsize[1])
        return box
    @abc.abstractmethod
    def clone(self,obj,assignId=False):
        cloner = Object(assignId=assignId, collisionShape=obj.getShape())
        cloner.pos = np.copy(obj.pos)
        cloner.rsize = np.copy(obj.rsize)
        return cloner

    def quickCheck2(self, a, b):
        isCollision = True
        selfShape =  a.getShape()
        otherShape = b.getShape()
        if selfShape == 2 and otherShape == 1:
            #isCollision, normal, contractPoint = self.circleRect3(a.getBoundingBox(),b.getBoundingBox())
            isCollision, normal,contractPoint= self.circleRect2(a.pos[0], a.pos[1], a.rsize[0], b.pos[0], b.pos[1], b.rsize[0], b.rsize[1])
        elif selfShape ==1 and otherShape ==1:
            isCollision,normal,contractPoint= self.rectRect(a,b)
        elif selfShape == 2 and otherShape ==2:
            isCollision,normal,contractPoint= self.circleCircle(a,b)

        normal = Utils.normalize(normal)
        self.normal = normal
        return isCollision,normal,contractPoint

    def circleRect3(self,boxa,boxb):
        return self.circleRect2(boxa.x,boxa.y,boxa.w,boxb.x,boxb.y,boxb.w,boxb.h)
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
        temp = dx**2 + dy**2
        #temp = dx**2 + dy**2

        #dRad = rad**2

        if temp <= rad**2:
            return True,np.array([cx-px,cy-py]),np.array([px,py])
        else:
            return False,np.array([0,0]),np.array([0,0])

    def circleCircle(self,a,b):
        dx = a.pos[0] - b.pos[0]
        dy = a.pos[1] - b.pos[1]
        d = np.array([dx, dy])
        temp = math.sqrt(dx ** 2 + dy ** 2)

        if temp <= a.rsize[0] + b.rsize[0]:
            return True,np.array([b.pos[0]-(a.pos[0]+a.rsize[0]),b.pos[1]-(a.pos[1]+a.rsize[1])]),(a.pos + b.pos)/2
        else:
            return False,np.array([0,0]),np.array([0,0])
    def rectRect(self, a, b):
        temp = not(a.pos[0] + a.rsize[0] < b.pos[0]
            or a.pos[0] > b.pos[0] + b.rsize[0]
            or a.pos[1] + a.rsize[1] < b.pos[1]
            or a.pos[1] > b.pos[1] + b.rsize[1] ),np.array([0,0])

        if temp:
            colTime, normalx, normaly =a.AABB(b)
            if colTime <1:
                return True,np.array([normalx,normaly]),np.array([0,0]),(a.pos + b.pos)/2

        return False,np.array([0,0]),np.array([0,0])

    def closestPointOnLine(self,lx1,ly1,lx2,ly2,x0,y0):
        A1 = ly2 - ly1
        B1 = lx1 - lx2
        C1 = (ly2-ly1)*lx1 + (lx1-lx2)*ly1
        C2 = -B1*x0 + A1 * y0
        det = A1*B1 - -B1*B1
        cx=0
        cy=0
        if det !=0 :
            cx = (A1*C1 - B1*C2)/det
            cy = (A1*C2 - -B1*C1)/det
        else:
            cx = x0
            cy = y0
        return [cx,cy]