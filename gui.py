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
import numpy as np

class MRS(QMainWindow):#(QWidget):#

    screenWidth = 1080
    screenHeight= 720

    def __init__(self):
        #QWidget.__init__(self, None)
        super(MRS, self).__init__()
        self.initGame()
        self.initUI()

    def initGame(self):
        self.walls = []
        self.robot = Robot()
        #self.wallLeft = Wall()
        #self.wallRight = Wall()
        #self.wallTop = Wall()
        #self.wallBot = Wall()

        centerX,centerY = self.getCenter()
        #self.wallMid =
        self.walls.append(Wall(centerX,centerY,100,100))
        self.walls.append(Wall(centerX-self.screenWidth/2, centerY, 20, self.screenHeight))
        self.walls.append(Wall(centerX+self.screenWidth/2, centerY, 20, self.screenHeight))
        self.walls.append(Wall(centerX, centerY-self.screenHeight/2, self.screenWidth, 20))
        self.walls.append(Wall(centerX, centerY+self.screenHeight/2, self.screenWidth, 20))

        self.doPress = False
        self.isCollided = False
        self.timer = QBasicTimer()

    def getCenter(self):
        return (self.screenWidth/2,self.screenHeight/2)

    def initUI(self):    
        '''initiates application UI'''

        #self.tboard = Board(self)
        #self.setCentralWidget(self.tboard)
        #self.statusbar = self.statusBar()   self.tboard.msg2Statusbar[str].connect(self.statusbar.showMessage)
        
        horizontalLayout = QHBoxLayout(self)
        verticalLayout = QVBoxLayout(self)
        verticalLayout.addLayout(horizontalLayout)
        #verticalLayout.addWidget(self.robot)
        self.setLayout(verticalLayout)
        self.resize(1080, 720)
        #self.center()
        self.setWindowTitle('Mobile Robot Simulator')        
        self.show()
        #self.worker = Worker(self)
        #self.worker.start()
        self.lastFrameTime = time.time()
        self.timer.start(17,self)

    def updateLogic(self,dt):
        #print("dt="+str(dt))


        '''
        if self.isCollided == False:
            box = self.robot.clone(assignId=False)
            box.pos = np.array(
                [self.robot.pos[0] + self.robot.getVelocity()[0], self.robot.pos[1] + self.robot.getVelocity()[1]])
            for wall in self.walls:
                if  box.checkCollision(wall):
                    self.isCollided = True
                    break
        '''
        #if not self.isCollided:
        prevPos = self.robot.pos

        self.isCollided = False

        for wall in self.walls:
            temp = self.robot.checkCollision(wall)
            if self.isCollided == False:
                self.isCollided = temp

        #if not self.isCollided:
        self.robot.updateTransform(dt)

        '''
        self.isCollided = False
        for wall in self.walls:
            if self.robot.checkCollision(wall,doResponse=False):
                self.isCollided = True
                break

        if self.isCollided:
            self.robot.pos= prevPos
        '''
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
            self.robot.vr +=  0.25
            self.robot.vl +=  0.25

        elif key == Qt.Key_G:
            print("pressG")
            self.doPress = True
            self.robot.vr +=  -0.25
            self.robot.vl +=  -0.25

        elif key == Qt.Key_X:
            print("pressX")
            #self.doPress = True
            self.robot.vr = 0
            self.robot.vl = 0

        if key == Qt.Key_C:
            self.robot.setPosition(500,500)
            self.robot.forward = np.array([1.,0.])
            self.robot.theta =0
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
        self.robot.draw(qp)
        for wall in self.walls:
            wall.draw(qp)
        #self.wallMid.draw(qp)
        #self.drawRobot(qp)
        qp.end()
        
        
    def drawLines(self, qp):
      
        pen = QPen(Qt.black, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawRect(4,4,550,450)

        pen2 = QPen(Qt.black, 1, Qt.SolidLine)
        qp.setPen(pen2)
        qp.drawRect(200,210,80,60)

    def drawRobot(self, qp):
      
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawEllipse(40,40,50,50)
        pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        qp.setPen(pen2)
        qp.drawLine(90,65,60,65)
        pen3 = QPen(Qt.blue, 0.5, Qt.SolidLine)
        qp.setPen(pen3)
        qp.drawLine(55,50,75,50)
        pen4 = QPen(Qt.black, 0.5, Qt.SolidLine)
        qp.setPen(pen4)
        qp.drawLine(55,80,75,80)

    #def center(self):
        #'''centers the window on the screen'''
        
        #screen = QDesktopWidget().screenGeometry()
        #size = self.geometry()
        #self.move((screen.width()-size.width())/2, 
            #(screen.height()-size.height())/2)

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
