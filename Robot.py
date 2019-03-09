
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from collections import namedtuple
import Utils
import numpy as np
from Object import Object
'''
@author Truong Huy Cuong
'''
class Robot(Object):
    def angle(v1, v2):
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        return angle

    def getVelocity(self):

        return self.forward * (self.vl + self.vr)/2

    def getShape(self):
        return 2#Object.Shape.CIRCLE

    def Reset(self,startPos):
        self.prevPos = np.zeros(2)
        self.maxVeloc = 5
        self.epsilon = 0.9
        self.point = np.array([0, 0])
        self.normal = np.array([0, 0])
        # main properties
        self.pos = np.copy(startPos)
        self.forward = np.array([1., 0.])
        self.vl = 0.0
        self.vr = 0.0
        self.v = (self.vl + self.vr) / 2
        self.size = 50.0
        self.rsize = np.array([self.size, self.size])
        self.l = 250
        self.ICC = self.pos
        self.rotation = 0
        self.R = 0
        xAxes = np.array([1, 0])
        self.theta = np.arccos(np.dot(self.forward, xAxes) / (
                np.linalg.norm(self.forward) * np.linalg.norm(xAxes)))  # self.angle(self.forward,np.array([1,0]))

        self.sThreshold = 200
        self.updateSensors()
        self.sDistances = np.full(len(self.sensors),self.sThreshold)  # np.zeros(len(self.robot.sensors))

        self.sEnableFlags = np.ones((len(self.sensors),),dtype=bool)
        self.isActive = True



    def __init__(self,startPos,parent=None,assignId=True):
        super(Robot, self).__init__(assignId=assignId)

        #print("myId: "+str(self.id))
        #debug params
        self.Reset(startPos)


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
            self.sensors.append(self.pos + temp*self.rsize[0])




    def setPosition(self,x,y):
        self.pos = np.array([x,y])

    def checkLinePoint(self,ps1,pe1,ps2,pe2):
        A1 = pe1[1] - ps1[1]
        B1 = ps1[0] - pe1[0]
        A2 = pe2[1] - ps2[1]
        B2 = ps2[0] - pe2[0]
        delta = A1 * B2 - A2 * B1
        if delta <= 0 or delta > np.linalg.norm(pe1-ps1): #parallel
            return None

        C2 = A2 * ps2[0] + B2 * ps2[1]
        C1 = A1 * ps1[0] + B1 * ps1[1]
        invdelta = 1. / delta

        ret = [ (B2*C1 - B1*C2)*invdelta, (A1*C2 - A2*C1)*invdelta ]
        #ret = [(B2 * C1 ) * invdelta, (A1 * C2 ) * invdelta]

        return ret

    def checkLinePoint3(self,a1,a2,b1,b2):
        intersection= np.array([0,0])

        b = a2-a1
        d = b2-b1

        dot = b[0]*d[1]-b[1]*d[0]
        if dot ==0:
            return None

        c = b1-a1
        t= (c[0]*d[1]-c[1]*d[0]) / dot
        if t<0 or t>1:
            return None

        u = (c[0]*b[1]-c[1]*b[0])/dot
        if u<0 or u>1:
            return None

        intersection = a1 + t * b
        return [intersection[0],intersection[1]]




    def checkLineRect3(self, ps, pe,rtl, rtr, rbl, rbr):
        denoms = []
        top = self.checkLinePoint3(ps, pe, rtl, rtr)
        if top != None:
            denoms.append(top)

        right = self.checkLinePoint3(ps, pe, rtr, rbr)
        if right != None:
            denoms.append(right)

        bot = self.checkLinePoint3(ps, pe, rbr, rbl)
        if bot != None:
            denoms.append(bot)

        left = self.checkLinePoint3(ps, pe, rbl, rtl)
        if left != None:
            if self.onRect(left, rtl, rbr):
                denoms.append(left)

        if len(denoms) == 0:
            return False, np.zeros((2,))
        else:
            vec = denoms - ps
            dist = np.linalg.norm(vec,axis=1)
            minIndex = np.argmin(dist)
            return True, denoms[minIndex]

    def checkLineRect2(self,ps,pe,rx,ry,rw,rh):
        rtl = np.array([rx - rw / 2, ry - rh / 2])
        rtr = np.array([rx + rw / 2, ry - rh / 2])
        rbl = np.array([rx - rw / 2, ry + rh / 2])
        rbr = np.array([rx + rw / 2, ry + rh / 2])

        denoms = []
        top = self.checkLinePoint3(ps,pe,rtl,rtr)
        if top !=None:
            #if self.onRect(top,rtl,rbr):
                denoms.append(top)

        right = self.checkLinePoint3(ps, pe, rtr, rbr)
        if right !=None:
            #if self.onRect(right, rtl, rbr):
                denoms.append(right)

        bot =self.checkLinePoint3(ps, pe, rbr, rbl)
        if bot !=None:
            #if self.onRect(bot, rtl, rbr):
                denoms.append(bot)

        left =self.checkLinePoint3(ps, pe, rbl, rtl)
        if left !=None:
            if self.onRect(left, rtl, rbr):
                denoms.append(left)

        if len(denoms)==0:
            return False,np.zeros((2,))
        else:
            points = np.copy(denoms)
            dist = np.linalg.norm(points-ps)
            minIndex = np.argmin(dist)
            return True,points[minIndex]


    def updateSensorInfo(self,tempPos,other):
        #if self.prevPos[0] != tempPos[0] or self.prevPos[1] != tempPos[1]:

        rtl = other.getDot(0)
        rtr = other.getDot(1)
        rbl = other.getDot(2)
        rbr = other.getDot(3)
        for i, sensor in enumerate(self.sensors):
            if self.sDistances[i] == self.sThreshold:
                norVec = Utils.normalizeByDivideNorm(sensor - tempPos)
                sensorThreshold = sensor + norVec * self.sThreshold

                doInteract, contractPoint = self.checkLineRect3(sensor, sensorThreshold, rtl, rtr, rbl, rbr)

                if doInteract:
                    self.sDistances[i] = np.linalg.norm(contractPoint - sensor)

    def checkCollision(self, other, doResponse=True):
        if doResponse:
            if self.prevPos[0] != self.pos[0] or self.prevPos[1] != self.pos[1]:
                rtl = other.getDot(0)
                rtr = other.getDot(1)
                rbl = other.getDot(2)
                rbr = other.getDot(3)
                for i,sensor in enumerate(self.sensors):
                    if self.sDistances[i] == self.sThreshold:
                        norVec = Utils.normalizeByDivideNorm(sensor-self.pos)
                        sensorThreshold = sensor+norVec*self.sThreshold

                        doInteract, contractPoint = self.checkLineRect3(sensor, sensorThreshold, rtl,rtr,rbl,rbr)

                        if doInteract:
                            self.sDistances[i] = np.linalg.norm(contractPoint-sensor)
        return super(Robot,self).checkCollision(other,doResponse)

    def checkCollisionWithNewPos(self,tempPos,wall,wasCollided):
        rminX = wall.projectOnto(wall.getDot(0), [1, 0])
        rmaxX = wall.projectOnto(wall.getDot(3), [1, 0])
        rminY = wall.projectOnto(wall.getDot(0), [0, 1])
        rmaxY = wall.projectOnto(wall.getDot(3), [0, 1])


        horRect=wall.getDot(0) - wall.getDot(1)
        verRect=wall.getDot(0) - wall.getDot(2)

        projHorTempPos = self.projectOnto(tempPos,horRect)
        projVerTempPos = self.projectOnto(tempPos,verRect)

        projHorSelfPos = self.projectOnto(self.pos,horRect)
        projVerSelfPos = self.projectOnto(self.pos,verRect)


        p1, p2 = wall.interactCirclePoint(projHorTempPos[0], projHorTempPos[1], tempPos[0], tempPos[1], self.rsize[0])
        p3, p4 = wall.interactCirclePoint(projVerTempPos[0], projVerTempPos[1], tempPos[0], tempPos[1], self.rsize[0])

        p5, p6 = wall.interactCirclePoint(projHorSelfPos[0], projHorSelfPos[1], self.pos[0], self.pos[1], self.rsize[0])
        p7, p8 =wall.interactCirclePoint(projVerSelfPos[0], projVerSelfPos[1], self.pos[0], self.pos[1], self.rsize[0])

        ps = np.array([p1,p2,p3,p4,p5,p6,p7,p8])
        '''
        ps = []
        for dot in wall.dots:
            p1, p2 = wall.interactCirclePoint(dot[0], dot[1],tempPos[0], tempPos[1], self.rsize[0])
            p3, p4 = wall.interactCirclePoint(dot[0], dot[1], self.pos[0], self.pos[1], self.rsize[0])
            ps.append(p1)
            ps.append(p2)
            ps.append(p3)
            ps.append(p4)
        if tempPos[0] != self.pos[0] or tempPos[1] != self.pos[1]:
            p5, p6 = wall.interactCirclePoint(self.pos[0], self.pos[1], tempPos[0], tempPos[1], self.rsize[0])
            ps.append(p5)
            ps.append(p6)
        
        ps = np.copy(ps)
        '''

        minXIndex = np.argmin(ps[:, 0])
        maxXIndex = np.argmax(ps[:, 0])

        minYIndex = np.argmin(ps[:, 1])
        maxYIndex = np.argmax(ps[:, 1])

        cminX = self.projectOnto(ps[minXIndex], [1, 0])
        cmaxX = self.projectOnto(ps[maxXIndex], [1, 0])

        cminY = self.projectOnto(ps[minYIndex], [0, 1])
        cmaxY = self.projectOnto(ps[maxYIndex], [0, 1])

        isCollided = False
        if not (rminX[0] > cmaxX[0] or cminX[0] > rmaxX[0]):
            #overlap X
            if not (rminY[1] > cmaxY[1] or cminY[1] > rmaxY[1]):
                #overlap Y
                isCollided =True
                #print("overlap on X,Y")
                minorOverlapX  = min(cmaxX[0],rmaxX[0]) - max(cminX[0],rminX[0])
                minorOverlapY  = min(cmaxY[1],rmaxY[1]) - max(cminY[1],rminY[1])
                if minorOverlapX < minorOverlapY:
                    #print("horizontal collision")
                    if  self.pos[0]< wall.pos[0]:  #cmaxX[0] < rmaxX[0]:
                       #print("left collision")
                        self.onCollisionWith(wall,-1,0, np.zeros(2),wasCollided)
                    else:
                        #print("right collision")
                        self.onCollisionWith(wall,1,0, np.zeros(2),wasCollided)
                else:
                    #print("vertical collision")
                    if self.pos[1]< wall.pos[1]:  # cmaxY[1] < rmaxY[1]:
                        #print("top collision")
                        self.onCollisionWith(wall,0,-1, np.zeros(2),wasCollided)
                    else:
                        #print("bot collision")
                        self.onCollisionWith(wall,0,1, np.zeros(2),wasCollided)
        return isCollided

    def onCollisionWith(self, obj, normalX, normalY, contractPoint,wasCollided):

        projection = np.array([0,0])
        #if normalX !=0 and normalY !=0:
        #    vec = np.array([0,0])
        #    self.vl = self.vr = 0
        #    return True
        #else:


        if normalX !=0:
            if normalX > 0:
                vec = np.array([0,1])
                self.pos[0] = obj.pos[0]+self.rsize[0] +obj.rsize[0]/2 +0.1
            else:
                vec = np.array([0,-1])
                self.pos[0] = obj.pos[0]-self.rsize[0] -obj.rsize[0]/2 -0.1
        else:
            if normalY > 0:
                vec = np.array([-1,0])
                self.pos[1] = obj.pos[1]+self.rsize[0] +obj.rsize[1]/2 +0.1
            else:
                vec = np.array([1,0])
                self.pos[1] = obj.pos[1]-self.rsize[0] -obj.rsize[1]/2 - 0.1

        if not wasCollided:
            planeVec = np.array([normalY, -normalX])
            normal = np.array([normalX, normalY])
            #print("fw = "+str(self.forward))
            r = self.forward - 2 * (np.dot(self.forward, normal) * normal)

            b = self.projectOnto(r, planeVec)
            magB = np.linalg.norm(b)
            if magB > 0.0001:
                bNorm = b / magB
                self.forward = np.copy(bNorm)
                # self.theta = np.arccos(np.dot(self.forward, np.array([1, 0])) / (
                #        np.linalg.norm(self.forward) * np.linalg.norm(
                #    np.array([1, 0]))))  # self.angle(self.forward,np.array([1,0]))

                slidingVec = self.pos + self.forward * (self.vl + self.vr) / 2

                if self.onRect(slidingVec,[0,0],[Utils.SCREEN_WIDTH,Utils.SCREEN_HEIGHT]):
                    self.pos = slidingVec





        #projVec = Object.projectOnto(self.forward,planeVec)

        #invertProjVec =



        '''
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
        '''
        #n =  Utils.normalizeByDivideNorm(contractPoint - self.pos)
        '''
        self.forward = vec
        lastTheta= np.rad2deg(self.theta)
        self.theta = np.arccos(np.dot(self.forward, np.array([1, 0])) / (
                np.linalg.norm(self.forward)*np.linalg.norm(np.array([1, 0]))))  # self.angle(self.forward,np.array([1,0]))

        temTheta = np.rad2deg(self.theta)

        if lastTheta <0:
             self.theta = -self.theta
        '''
        #self.vl = self.vr = 0
        return True

    def setVelocity(self,vLeft,vRight):
        if vLeft >= 0 :
            self.vl = min(vLeft,self.maxVeloc)
        else:
            self.vl = max(vLeft,-self.maxVeloc)

        if vRight >= 0:
            self.vr = min(vRight, self.maxVeloc)
        else:
            self.vrvl = max(vRight, -self.maxVeloc)

    #box = namedtuple('Box',['x','y','w','h'])
    #Box = namedtuple('Box','x y w h')
    def getBoundingBox(self):
        delta = self.pos - self.prevPos
        box = Object.Box(self.prevPos[0], self.prevPos[1],delta[0] +self.rsize[0],delta[1]+self.rsize[1])
        return box

    def updateTransform(self,dt):

        tempPos =  np.copy(self.pos)
        self.prevPos = np.copy(self.pos)

        xAxes = np.array([1, 0])
        delta = self.vr - self.vl
        if delta < -self.epsilon or delta > self.epsilon:
            self.rotation = delta / self.l
            self.R = self.l/2 * (self.vr + self.vl)/delta
            #temp1 = self.vl * xAxes#self.forward
            #temp2 = self.vr * xAxes#self.forward
            #print("rotation = "+str(self.rotation)+",theta = "+str(self.theta))
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
            retMatrix =  np.dot(rotMatrix,toOriginMatrix) + backLocationMatrix

            #self.pos = np.array([retMatrix[0],retMatrix[1]])
            tempPos = np.array([retMatrix[0],retMatrix[1]])
            self.theta = retMatrix[2]
            self.forward = np.array(
                [np.cos(self.theta)*xAxes[0] - np.sin(self.theta)*xAxes[1],
                    np.sin(self.theta) * xAxes[0] + np.cos(self.theta) * xAxes[1],
                ])

            self.forward /= np.linalg.norm(self.forward)


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

            if self.vl > 0:
                signal = 1
            else:
                signal = -1

            self.rotation =0
            self.ICC = np.copy(self.pos)
            tempPos = self.pos + f * signal * self.vl**2 * dt





        return tempPos

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
        origin = np.array([self.pos[0] - self.rsize[0],self.pos[1] - self.rsize[0]])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawEllipse(origin[0], origin[1], self.rsize[0]*2, self.rsize[0]*2)

        #qp.drawPie(QRectF(origin[0], origin[1], self.size, self.size), 0, 5760)
        #forward
        pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        qp.setPen(pen2)
        f = np.copy(self.forward ) #Utils.normalize(self.forward)
        qp.drawLine(self.pos[0], self.pos[1], self.pos[0]+f[0]*self.size, self.pos[1]+f[1]*self.size)



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
        qp.drawText(QPointF(lefTextPos[0] ,lefTextPos[1] ),str("{:.2f}".format(self.vl)))

        rightTextPos = self.pos + pVec * -50
        pen = QPen(Qt.red, 1, Qt.SolidLine)
        qp.setPen(pen)

        # qp.drawPoint(textPos[0], textPos[1])
        qp.drawText(QPointF(rightTextPos[0], rightTextPos[1]),str("{:.2f}".format(self.vr)))



        for i,sensor in enumerate(self.sensors) :
            #pen6 = QPen(Qt.darkCyan, 0.01, Qt.SolidLine)
            pen6 = QPen(Qt.darkCyan, 1, Qt.SolidLine)
            qp.setPen(pen6)
            norVec = Utils.normalizeByDivideNorm(sensor - self.pos) *self.sThreshold
            qp.drawText(QPointF(sensor[0] +norVec[0], sensor[1]+norVec[1]), str("{:.1f}".format(self.sDistances[i])))
            #qp.drawLine(sensor[0], sensor[1], sensor[0] + norVec[0], sensor[1] + norVec[1])



        return True
