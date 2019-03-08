import numpy as np
from Object import Object
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import Utils
import pygame as pg
'''
@author Truong Huy Cuong
'''
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

        self.dots = []
        self.dots.append(np.array([x-width/2,y-height/2]))
        self.dots.append(np.array([x+width/2,y-height/2]))
        self.dots.append(np.array([x-width/2,y+height/2]))
        self.dots.append(np.array([x+width/2,y+height/2]))

    def getDot(self,i):
        return self.dots[i]

    def getNormal(self,point):
        normal = np.zeros(2)
        TOP_LEFT = 0
        TOP_RIGHT = 1
        BOT_LEFT= 2
        BOT_RIGHT =3
        if point[0] >=  self.dots[TOP_LEFT][0] and point[0]<= self.dots[TOP_RIGHT][0]:
            if point[1] == self.dots[TOP_LEFT][1]:
                normal[1] = -1
            elif point[1] == self.dots[BOT_LEFT][1]:
                normal[1] = 1
        if point[1] >= self.dots[TOP_LEFT][1] and point[1] <= self.dots[BOT_LEFT][1]:
            if point[0] == self.dots[TOP_LEFT][0]:
                normal[0] = -1
            elif point[0] ==self.dots[BOT_RIGHT][0]:
                normal[0] = 1

        return normal

    def checkOnRect(self,point):
        rtl = self.dots[0]
        rbr = self.dots[3]
        return  not(point[0] <rtl[0] or point[0] > rbr[0] or
                    point[1] < rtl[1] or point[1] > rbr[1]
        )

    def setSize(self,width,height):
        self.rsize = np.array([width,height])

    def draw(self,qp):
        origin = np.array([self.pos[0] - self.rsize[0] / 2, self.pos[1] - self.rsize[1] / 2])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawRect(origin[0],origin[1],self.rsize[0],self.rsize[1])

    def draw3(self,screen):
        origin = np.array([self.pos[0] - self.rsize[0] / 2, self.pos[1] - self.rsize[1] / 2])
        pg.draw.rect(screen, Utils.BLACK, [origin[0], origin[1], self.rsize[0], self.rsize[1]], 2)

