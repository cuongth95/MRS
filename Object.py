import abc
from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
class Object:#(QtWidgets.QWidget):
    #static things
    idCounter=0
    @staticmethod
    def getId(self):
        ret = self.idCounter + 1
        self.idCounter+=1
        return ret

    #constructor
    def __init__(self,parent=None):
        #QtWidgets.QWidget.__init__(self, parent)
        self.id = self.getId(self)
        self.pos = np.array([0,0])
        self.forward = np.array([1,0]) #right

    def setX(self,value):
        self.pos[0] = value

    def setY(self,value):
        self.pos[1] = value

    @abc.abstractmethod
    def draw(self,qp):
        return True

    @abc.abstractmethod
    def checkCollision(self,other):
        return True

