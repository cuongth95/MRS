
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen, QBrush

'''
@author Truong Huy Cuong
'''

class Beacon:
    idCounter = 0
    @staticmethod
    def AutoGenId():
        index = Beacon.idCounter+ 1
        Beacon.idCounter +=1
        return index

    def __init__(self,startPos,doGenId=True):
        if doGenId:
            self.id = Beacon.AutoGenId()
        else:
            self.id = -1

        self.size = 10
        self.rsize = np.array([self.size,self.size])
        self.pos = np.copy(startPos)
        #feature-measurements
        self.isDetected = False





    def Draw(self,qp):
        origin = np.array([self.pos[0] - self.rsize[0], self.pos[1] - self.rsize[0]])
        if not self.isDetected:
            brush = QBrush(Qt.darkBlue)
        else:
            brush = QBrush(Qt.darkGreen)
        qp.setBrush(brush)
        qp.drawEllipse(origin[0], origin[1], self.rsize[0] * 2, self.rsize[0] * 2)
        qp.drawText(self.pos[0],self.pos[1],str(self.id))