from PyQt5.QtCore import Qt, QPointF, QBasicTimer
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtWidgets import QMainWindow, QApplication
from LRobot import LRobot
from Beacon import Beacon
from KalmanFilter import KalmanFilter
import time
import Utils
import sys
import numpy as np
import Map
'''
@author Truong Huy Cuong
'''

class MobileRobotLocalization(QMainWindow):
    screenWidth = Utils.SCREEN_WIDTH
    screenHeight= Utils.SCREEN_HEIGHT

    def __init__(self,enableGUI):
        self.InitGame()
        if enableGUI:
            super(MobileRobotLocalization, self).__init__()
            self.InitGUI()


    def InitGUI(self):
        print("init gui")
        self.timer = QBasicTimer()
        self.lastFrameTime =0
        self.resize(self.screenWidth, self.screenHeight)
        self.setWindowTitle('Mobile Robot Localization')
        self.show()
        self.timer.start(1000.0 / 60, self)

    def InitGame(self):
        print("init game")
        self.fps = 30.0

        self.beacons = []
        self.lines = []
        self.timeCounter=0
        self.timeDelayDrawing = 2
        #beacons pos
        self.startPos,self.beacons,self.lines = Map.genAssignmentMap()
        self.pRobot = LRobot(self.startPos, Qt.magenta, True)
        self.cRobot = LRobot(self.startPos, Qt.green, True)
        self.robot = LRobot(self.startPos)

        #parameters
        self.lastMean = np.array([self.robot.pos[0],self.robot.pos[1],self.robot.theta])
        self.lastCovariance = np.array([0.1,0.1,0.1])


    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_W:
            print("pressW")
            self.robot.v +=1.0
        elif key == Qt.Key_S:
            print("pressS")
            self.robot.v -=1.0
        elif key == Qt.Key_A:
            print("pressA")
            self.robot.rotation -=0.05
        elif key == Qt.Key_D:
            print("pressD")
            self.robot.rotation +=0.05
        elif key == Qt.Key_X:
            print("pressX")
            self.robot.rotation =0.0
            self.robot.v =0.0

        if key == Qt.Key_C:
            self.timeCounter = 0
            self.robot.closePaths()
            self.robot.Reset(startPos= self.startPos)
            self.cRobot.Reset(startPos= self.startPos)
            self.pRobot.Reset(startPos= self.startPos)
            self.lastMean = np.array([self.robot.pos[0], self.robot.pos[1], self.robot.theta])
            self.lastCovariance = np.array([0.1, 0.1, 0.1])
            return None

    def mouseReleaseEvent(self, event):
        #print("im herre - pos ="+event.pos)
        if event.button() == Qt.LeftButton:
            mpos = event.pos()
            print(str(mpos))
            self.beacons.append(Beacon([mpos.x(),mpos.y()]))

    def UpdateLogic(self,dt):
        #print("Update Logic")
        if dt >0.001:
            self.fps = 1.0 /dt

        self.timeCounter += dt



        #update beacons
        self.robot.features = []
        for beacon in self.beacons:
            beacon.isDetected = self.robot.UpdateSensor(beacon)

        #missing feature-measurement

        pMean,pCovariance,mean,covariance = KalmanFilter(dt,
                     self.robot.theta,
                     self.lastMean,
                     self.lastCovariance,
                     np.array([self.robot.v,self.robot.rotation]),
                     self.robot.GetMeasurement()
                     )
        self.lastMean = np.copy(mean)
        self.lastCovariance = np.copy(covariance)
        self.cRobot.UpdateTransform([mean[0], mean[1]], mean[2])
        print("estimated pos = "+str([mean[0],mean[1]]))
        #guessPos = self.robot.GetMeasurement()

        #self.pRobot.UpdateTransform([pMean[0],pMean[1]],pMean[2])
        #self.robot.AddEstimatedPath(mean, covariance, True)

        if self.timeCounter > self.timeDelayDrawing:
            self.timeCounter =0
            self.robot.AddEstimatedPath(mean, covariance, True)
        else:
            self.robot.AddEstimatedPath(mean, covariance, False)

        # calculate transform but not apply
        pos, theta = self.robot.CalculateTransform(dt)
        # apply them using that func
        self.robot.UpdateTransform(pos, theta)


        #temp =self.robot.GetMeasurement()
        #self.pRobot.UpdateTransform([temp[0], temp[1]], temp[2])

    def Draw(self,qpainter):
        #print("render func")

        self.robot.Draw(qpainter)
        self.cRobot.Draw(qpainter)
        #self.pRobot.Draw(qpainter)

        pen = QPen(Qt.black, 1.5, Qt.SolidLine)
        qpainter.setPen(pen)
        for line in self.lines:
            p1 = self.beacons[line[0]-1].pos
            p2 = self.beacons[line[1]-1].pos
            qpainter.drawLine(p1[0],p1[1],p2[0],p2[1])

        for beacon in self.beacons:
            beacon.Draw(qpainter)

        #pen = QPen(Qt.black, 5, Qt.SolidLine)

        pen = QPen(Qt.black, 1.5, Qt.SolidLine)
        qpainter.setPen(pen)
        qpainter.drawText(QPointF(10, 50), "FPS: " + str("{:.2f}".format(self.fps)))




    # system functions - no need to take care :D
    def timerEvent(self, event):
        if event.timerId() == self.timer.timerId():
            currentTime = time.time()
            dt = (currentTime - self.lastFrameTime)
            self.lastFrameTime = currentTime
            self.UpdateLogic(dt)
            self.update()
        else:
            super(MobileRobotLocalization, self).timerEvent(event)

    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        self.Draw(qp)
        qp.end()

if __name__ == '__main__':
    QApplication.processEvents()

    app = QApplication([])
    game = MobileRobotLocalization(enableGUI=True)

    sys.exit(app.exec_())
