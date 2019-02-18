import numpy as np
from Object import Object
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Wall(Object):
    def getVelocity(self):
        return np.array([0,0])
    def getShape(self):
        return 1#shape.SQUARE
    def __init__(self,x,y,width,height,parent=None):
        super(Wall, self).__init__()
        self.rsize = np.array([width,height])
        self.pos = np.array([x,y])
        self.normal = np.array([0,0])

    def setSize(self,width,height):
        self.rsize = np.array([width,height])

    def draw(self,qp):
        origin = np.array([self.pos[0] - self.rsize[0] / 2, self.pos[1] - self.rsize[1] / 2])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawRect(origin[0],origin[1],self.rsize[0],self.rsize[1])

