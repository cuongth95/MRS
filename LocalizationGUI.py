from PyQt5.QtCore import Qt, QPointF, QBasicTimer
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import QMainWindow, QApplication
from LRobot import LRobot
from Beacon import Beacon
from KalmanFilter import KalmanFilter
import time
import Utils
import sys
import numpy as np
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
        self.pRobot = LRobot([50,50],Qt.magenta,True)
        self.robot = LRobot([50,50])
        self.beacons = []
        self.lines = []
        #beacons pos
        self.beacons.append(Beacon([0,0]))
        self.beacons.append(Beacon([0,144]))
        self.beacons.append(Beacon([0,144*5]))
        self.beacons.append(Beacon([360,288]))
        self.beacons.append(Beacon([360,144*4]))
        self.beacons.append(Beacon([360*2,144]))
        self.beacons.append(Beacon([360*2,144*3]))
        self.beacons.append(Beacon([360*2,144*5]))
        self.beacons.append(Beacon([360*3,0]))
        self.beacons.append(Beacon([360*3,144*2]))
        self.beacons.append(Beacon([360*3,144*5]))
        #TEMP lines connection between beacons
        self.lines.append([1, 5])
        self.lines.append([6, 7])
        self.lines.append([2, 7])
        self.lines.append([3, 4])
        self.lines.append([0, 1])
        self.lines.append([1, 2])
        self.lines.append([0,8])
        self.lines.append([8,9])
        self.lines.append([3,9])
        self.lines.append([9,10])
        self.lines.append([7,10])

        #parameters
        self.lastMean = np.ones(3)
        self.lastCovariance = 0

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
            self.robot.rotation -=0.1
        elif key == Qt.Key_D:
            print("pressD")
            self.robot.rotation +=0.1
        elif key == Qt.Key_X:
            print("pressX")
            self.robot.rotation =0.0
            self.robot.v =0.0

        if key == Qt.Key_C:
            self.robot.Reset(startPos=[50,50])
            self.pRobot.Reset(startPos=[50,50])
            return None



    def UpdateLogic(self,dt):
        #print("Update Logic")
        if dt >0.001:
            self.fps = 1.0 /dt
        # calculate transform but not apply
        pos,theta = self.robot.CalculateTransform(dt)
        # apply them using that func
        self.robot.UpdateTransform(pos,theta)

        #missing feature-measurement
        mean,covariance = KalmanFilter(dt,
                     theta,
                     self.lastMean,
                     self.lastCovariance,
                     np.array([self.robot.v,self.robot.rotation]),
                     )



    def Draw(self,qpainter):
        #print("render func")

        self.robot.Draw(qpainter)
        self.pRobot.Draw(qpainter)
        for line in self.lines:
            p1 = self.beacons[line[0]].pos
            p2 = self.beacons[line[1]].pos
            qpainter.drawLine(p1[0],p1[1],p2[0],p2[1])

        for beacon in self.beacons:
            beacon.Draw(qpainter)

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
