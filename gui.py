import sys
import time
import random
from PyQt5.QtWidgets import QMainWindow, QFrame, QDesktopWidget, QApplication
from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog,
QDialogButtonBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
QLabel, QLineEdit, QMenu, QMenuBar, QPushButton, QSpinBox, QTextEdit,
QVBoxLayout,QWidget)
from PyQt5.QtCore import Qt, QBasicTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor 
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from Robot import Robot
from Wall import  Wall
from Object import Object
import Evolution
import numpy as np
import Utils


class MRS(QMainWindow):#(QWidget):#

    screenWidth = Utils.SCREEN_WIDTH
    screenHeight= Utils.SCREEN_HEIGHT

    def __init__(self):
        self.frameRate = 60
        #QWidget.__init__(self, None)
        super(MRS, self).__init__()
        self.initGame()
        self.initUI()

    def initGame(self):
        self.cminY = np.zeros(2)
        self.cmaxY = np.zeros(2)
        self.cminX = np.zeros(2)
        self.cmaxX = np.zeros(2)

        self.box = None
        self.walls = []
        self.robot = Robot(startPos=[500,600])
        #self.wallLeft = Wall()
        #self.wallRight = Wall()
        #self.wallTop = Wall()
        #self.wallBot = Wall()

        centerX,centerY = self.getCenter()
        #self.wallMid =
        self.walls.append(Wall(centerX,centerY,300,300))
        self.walls.append(Wall(centerX-self.screenWidth/2, centerY, 20, self.screenHeight))
        self.walls.append(Wall(centerX+self.screenWidth/2, centerY, 20, self.screenHeight))
        self.walls.append(Wall(centerX, centerY-self.screenHeight/2, self.screenWidth, 20))
        self.walls.append(Wall(centerX, centerY+self.screenHeight/2, self.screenWidth, 20))

        self.doPress = False
        self.isCollided = False
        self.timer = QBasicTimer()
        self.prevPos = self.robot.pos

        self.robot.updateSensors()
        self.robot.sDistances = np.full(len(self.robot.sensors),self.robot.sThreshold)#np.zeros(len(self.robot.sensors))
        for wall in self.walls:
            self.robot.updateSensorInfo(self.robot.pos, wall)

        self.ea = Evolution.Evolution(self,[540,600],200)



    def getCenter(self):
        return (self.screenWidth/2,self.screenHeight/2)

    def initUI(self):    
        '''initiates application UI'''

        #self.tboard = Board(self)
        #self.setCentralWidget(self.tboard)
        #self.statusbar = self.statusBar()   self.tboard.msg2Statusbar[str].connect(self.statusbar.showMessage)

        verticalLayout = QVBoxLayout()
        #verticalLayout.addWidget(self.robot)
        self.setLayout(verticalLayout)
        self.resize(1080, 720)
        #self.center()
        self.setWindowTitle('Mobile Robot Simulator')        
        self.show()
        #self.worker = Worker(self)
        #self.worker.start()
        self.lastFrameTime = time.time()

        self.timer.start(1000.0/60,self)

    '''
    def checkCollision(self,r,doResponse=True):
        collideFlag = False

        ##if r.pos[0] <0-50 or r.pos[0] > self.screenWidth+50 or r.pos[1]<0-50 or r.pos[1] > self.screenHeight+50:
        #    print("quach thi phung")
        for wall in self.walls:
            temp = r.checkCollision(wall,doResponse)
            if collideFlag == False:
                collideFlag = temp

        return collideFlag
    '''


    def updateLogic(self,dt):

        self.frameRate = dt * 1000

        #self.robot.vl=self.robot.vr = 25
        #self.robot.pos = np.array([1020,500])

        if not self.isCollided:
            tempPos = self.robot.updateTransform(dt)
        else:
            tempPos = np.copy(self.robot.pos)

        #tempPos = np.array([1056.009,500])
        self.isCollided = False
        for wall in self.walls:
            flag = self.robot.checkCollisionWithNewPos(tempPos,wall,self.isCollided)
            if not self.isCollided:
                self.isCollided = flag


        if not self.isCollided:
            self.robot.pos = tempPos

        if self.robot.prevPos[0] != self.robot.pos[0] or self.robot.prevPos[1] != self.robot.pos[1]:
            self.robot.updateSensors()
            self.robot.sDistances = np.full(len(self.robot.sensors),self.robot.sThreshold)#np.zeros(len(self.robot.sensors))
            for wall in self.walls:
                self.robot.updateSensorInfo(self.robot.pos, wall)


        #self.ea.updateLogic(dt)
        return True





    def keyPressEvent(self, event):
        key = event.key()

        prevVl = self.robot.vl
        prevVr = self.robot.vr
        if key == Qt.Key_W:
            print("pressW")
            self.doPress = True
            self.robot.vl += 5  # self.robot.forward * 5#
        elif key == Qt.Key_S:
            print("pressS")
            self.doPress = True
            self.robot.vl += -5  # self.robot.forward * -5#
        elif key == Qt.Key_O:
            print("pressO")
            self.doPress = True
            self.robot.vr += 5  # self.robot.forward * 5
        elif key == Qt.Key_L:
            print("pressL")
            self.doPress = True
            self.robot.vr +=  -5  # self.robot.forward * -5#
        elif key == Qt.Key_T:
            print("pressT")
            self.doPress = True
            self.robot.vr += 5
            self.robot.vl += 5

        elif key == Qt.Key_G:
            print("pressG")
            self.doPress = True
            self.robot.vr +=  -5
            self.robot.vl +=  -5

        elif key == Qt.Key_X:
            print("pressX")
            #self.doPress = True
            self.robot.vr = 0
            self.robot.vl = 0

        if key == Qt.Key_C:
            self.robot.Reset(startPos=[500,500])
            return None

        if self.doPress:
            self.doPress = False

            '''
            if not self.isCollided:
                box = self.robot.clone(assignId=False)
                #box.updateTransform(0.17)
                if box.checkCollision(self.wallMid,doResponse=False):
                    self.robot.vl=0
                    self.robot.vr=0
            '''
            print("vl= " + str(self.robot.vl) + ",vr=" + str(self.robot.vr))
            #self.robot.updateTransform(1)
            #self.robot.updatePosition(1)


        #self.robot.updateTransform(1)
        # self.robot.vl = 0
        #self.robot.vr = 0

    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)



        #self.drawLines(qp)

        #origin = np.array([self.tempPos[0] - self.robot.rsize[0], self.tempPos[1] -  self.robot.rsize[0]])
        pen = QPen(Qt.green, 3.5, Qt.SolidLine)
        qp.setPen(pen)
        #qp.drawLine(self.robot.pos[0],self.robot.pos[1] ,self.robot.pos[0]+self.tempPos[0]*100,self.robot.pos[1]+self.tempPos[1]*100)
        #qp.drawLine(self.cminX[0], self.cminX[1], self.cmaxX[0], self.cmaxX[1])

        #qp.drawLine(self.cminY[0], self.cminY[1], self.cmaxY[0], self.cmaxY[1])
        #print("robotpos = "+str(self.robot.pos))
        self.robot.draw(qp)

        #self.ea.draw(qp)

        for wall in self.walls:
            wall.draw(qp)
        #self.wallMid.draw(qp)
        #self.drawRobot(qp)

        pen = QPen(Qt.black, 5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawText(QPointF(100, 250), "Frame rate: " + str("{:.2f}".format(self.frameRate)))
        qp.end()
        

    def timerEvent(self, event):
        if event.timerId() == self.timer.timerId():
            currentTime = time.time()
            dt = (currentTime - self.lastFrameTime)
            self.lastFrameTime = currentTime
            self.updateLogic(dt)
            self.update()
        else:
            super(MRS, self).timerEvent(event)

class Worker(QThread):

    #updateProgress = QtCore.Signal(int)

    def __init__(self,g):
        QThread.__init__(self)
        self.game = g

    FPS = 60
    isRunning = True
    lastFrameTime = 0
    #A QThread is run by calling it's start() function, which calls this run()
    #function in it's own "thread".
    def run(self):
        #Notice this is the same thing you were doing in your progress() function
        while self.isRunning:
            currentTime = time.time()
            dt = (currentTime - self.lastFrameTime)
            self.lastFrameTime = currentTime
            sleepTime = 1.0 / self.FPS - dt
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                #print("dt:"+str(dt*1000))
                #self.game.processEvents()
                self.game.updateLogic(dt)

                #self.game.processEvents()
                #self.game.render()
            self.game.update()



if __name__ == '__main__':
    QApplication.processEvents()

    app = QApplication([])
    game = MRS()

    sys.exit(app.exec_())
