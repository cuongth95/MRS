import sys
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


class MRS(QMainWindow):
    
    def __init__(self):
        super(MRS, self).__init__()
        self.initUI()
        
        
    def initUI(self):    
        '''initiates application UI'''

        #self.tboard = Board(self)
        #self.setCentralWidget(self.tboard)
        #self.statusbar = self.statusBar()   self.tboard.msg2Statusbar[str].connect(self.statusbar.showMessage)
        
        horizontalLayout = QHBoxLayout(self)
        verticalLayout = QVBoxLayout(self)
        verticalLayout.addLayout(horizontalLayout)
        self.setLayout(verticalLayout)
        
        self.resize(700, 500)
        #self.center()
        self.setWindowTitle('Mobile Robot Simulator')        
        self.show()
        
    def paintEvent(self, e):

        qp = QPainter()
        qp.begin(self)
        self.drawLines(qp)
        self.drawRobot(qp)
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
        

   


if __name__ == '__main__':
    app = QApplication([])
    mygui = MRS()    
    sys.exit(app.exec_())
