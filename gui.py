import sys
import time
import random
from PyQt5.QtWidgets import QMainWindow, QFrame, QDesktopWidget, QApplication
from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog,
QDialogButtonBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
QLabel, QLineEdit, QMenu, QMenuBar, QPushButton, QSpinBox, QTextEdit,
QVBoxLayout)
from PyQt5.QtCore import Qt, QBasicTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor 
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from Robot import Robot

class MRS(QMainWindow):
    
    def __init__(self):
        super(MRS, self).__init__()
        self.initGame()
        self.initUI()

    def initGame(self):
        self.robot = Robot()
        self.doPress = False

        
    def initUI(self):    
        '''initiates application UI'''

        #self.tboard = Board(self)
        #self.setCentralWidget(self.tboard)
        #self.statusbar = self.statusBar()   self.tboard.msg2Statusbar[str].connect(self.statusbar.showMessage)
        
        horizontalLayout = QHBoxLayout(self)
        verticalLayout = QVBoxLayout(self)
        verticalLayout.addLayout(horizontalLayout)
        self.setLayout(verticalLayout)
        
        self.resize(1080, 720)
        #self.center()
        self.setWindowTitle('Mobile Robot Simulator')        
        self.show()
        self.worker = Worker(self)
        self.worker.start()

    def updateLogic(self,dt):
        return True

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            print("pressW")
            self.doPress = True
            self.robot.vl += 5  # self.robot.forward * 5#
        elif key == Qt.Key_S:
            print("pressS")
            self.doPress = True
            self.robot.vl += -5  # self.robot.forward * -5#
        if key == Qt.Key_O:
            print("pressO")
            self.doPress = True
            self.robot.vr += 5  # self.robot.forward * 5
        elif key == Qt.Key_L:
            print("pressL")
            self.doPress = True
            self.robot.vr += -5  # self.robot.forward * -5#

        if self.doPress:
            print("vl= " + str(self.robot.vl) + ",vr=" + str(self.robot.vr))
            self.doPress = False
            self.robot.updatePosition(1)
        #self.robot.updatePosition(1)
        # self.robot.vl = 0
        #self.robot.vr = 0

    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)

        self.drawLines(qp)
        self.robot.draw(qp)
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
    app = QApplication([])
    mygui = MRS()    
    sys.exit(app.exec_())
