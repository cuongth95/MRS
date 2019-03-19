from PyQt5.QtCore import Qt, QPointF, QBasicTimer
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import QMainWindow, QApplication
import time

from LRobot import LRobot
import Utils
import sys
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
        self.robot = LRobot([self.screenWidth/2,self.screenHeight/2])


    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_W:
            print("pressW")
            self.robot.v +=5.0
        elif key == Qt.Key_S:
            print("pressS")
            self.robot.v -=5.0
        elif key == Qt.Key_A:
            print("pressA")
            self.robot.rotation -=0.5
        elif key == Qt.Key_D:
            print("pressD")
            self.robot.rotation +=0.5
        elif key == Qt.Key_X:
            print("pressX")
            self.robot.rotation =0.0
            self.robot.v =0.0

        if key == Qt.Key_C:
            self.robot.Reset(startPos=[500,500])
            return None



    def UpdateLogic(self,dt):
        #print("Update Logic")
        self.fps = 1.0 /dt
        # calculate transform but not apply
        pos,theta = self.robot.CalculateTransform(dt)
        # apply them using that func
        self.robot.UpdateTransform(pos,theta)

    def Draw(self,qpainter):
        #print("render func")
        qpainter.drawText(QPointF(10, 50), "FPS: " + str("{:.2f}".format(self.fps)))
        self.robot.Draw(qpainter)

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
