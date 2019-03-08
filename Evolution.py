from hmac import new

from PyQt5.QtCore import QPointF

from gui import MRS
from Robot import Robot
from collections import namedtuple
import numpy as np
'''
@author Truong Huy Cuong
'''
def sigmoid(z):
    ret = 1.0 / (1.0 + np.exp(-z))
    return ret
def softPlus(z):
    #ret = - np.log(1.0 - 1.0/(1.0 + np.exp(-z)))
    ret = np.log(1 + np.exp(z))
    #ret = np.max(0,z)
    return ret

def RNNInit(input,hidden,
                   outputN=2):
    x = np.concatenate((input, hidden))
    xN = len(x)
    hiddenN = len(hidden)
    b1 = np.ones((hiddenN,))
    b2 = np.ones((outputN,))
    W1 = np.random.rand(hiddenN,xN) *0.1
    W2 = np.random.rand(outputN,hiddenN) *0.1

    return RNN(x,b1,W1,b2,W2)


def FeedForward(RNN,activation=sigmoid):
    return InternalFeedForward(RNN.x,RNN.b1,RNN.W1,RNN.b2,RNN.W2,activation)

def InternalFeedForward(x,b1,W1,b2,W2,activation=softPlus):
    # activation in layer 1
    a1 = x.T
    # activation in layer 2
    z2 = np.dot(W1, a1) + b1
    a2 = softPlus(z2)
    # activation in layer 3
    z3 = np.dot(W2, a2) + b2
    a3 = softPlus(z3)
    return a2,a3

RNN =  namedtuple('RNN',['x','b1','W1','b2','W2'])

class Evolution:

    def __init__(self,gui,startPos,sThreshold):


        self.gui = gui
        #self.mainRobot = gui.robot
        self.mutationRate = 0.1
        self.startPos = startPos
        self.Initilization(sThreshold)


    def Initilization(self,threshold,population=20):
        self.curIndex=0
        self.population = population
        self.numExistRobots = population
        self.maxFitness = 0
        self.cycle =0
        hiddenNodes = 4
        self.maxRange = threshold
        self.gens = np.random.rand(population,4+64+2+8) *0.001 #np.random.randint(self.maxRange,size = (population,4+64+2+8)) #
        # self.gens = np.concatenate((self.gens, np.zeros([hiddenNodes])))
        self.robots = []
        self.rnns = []
        #self.prevPoses = []
        for i in range(population):
            temp = Robot(assignId=False)
            temp.pos = np.copy(self.startPos)
            temp.updateSensors()
            temp.sDistances = np.zeros(len(temp.sensors))
            for wall in self.gui.walls:
                temp.updateSensorInfo(temp.pos, wall)
            #self.gui.checkCollision(temp) #update sensors
            self.robots.append(temp)
            hid = np.zeros(4)
            tempRNN = self.GetRNNParamsByGen(self.gens[i],temp,hid)

            #self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], temp, a2)
            self.rnns.append(tempRNN)#RNNInit(inp,hid))
            #self.gens.append(np.copy(temp.sDistances))
            #self.prevPoses.append(np.copy(temp.pos))




    def fitness(self,r):
        f = np.zeros((len(r), 1))

        for i, robot in enumerate(r):
            if robot.isActive:
                f[i] = np.linalg.norm(robot.pos -self.startPos) * (robot.vl + robot.vr)/2
            else:
                f[i] = np.linalg.norm(robot.pos - self.startPos)



        return f

    def GetRNNParamsByGen(self,gen,robot,hidden):

        x = np.array(np.concatenate((robot.sDistances,hidden)))

        weightsInLayer1 = gen[0:68]
        weightsInLayer2 = gen[68:]

        b1 = weightsInLayer1[0:4]
        W1 = weightsInLayer1[4:]
        W1 = np.reshape(W1,(4,12+4))

        b2 = weightsInLayer2[0:2]
        W2 = weightsInLayer2[2:]
        W2 = np.reshape(W2,(2,4))
        return RNN( x,b1,W1,b2,W2)



    speedUpAmount = 500
    timeCounter = 0
    timeLimit = 10 *speedUpAmount
    isCollided = False
    def updateLogicForAll(self, dt):
        isExisted = False
        self.numExistRobots = self.population
        for i, robot in enumerate(self.robots):
            if not robot.isActive:
                self.numExistRobots -= 1
                continue
            isExisted = True
            # assign to RNNs
            a2, a3 = FeedForward(self.rnns[i])
            robot.setVelocity(a3[0], a3[1])


            if not self.isCollided:
                tempPos = robot.updateTransform(dt)
            else:
                tempPos = np.copy(robot.pos)

            self.isCollided = False

            for wall in self.gui.walls:
                flag = robot.checkCollisionWithNewPos(tempPos, wall)
                if not self.isCollided:
                    self.isCollided = flag

            if not self.isCollided:
                robot.pos = tempPos
                for wall in self.gui.walls:
                    robot.updateSensorInfo(tempPos, wall)
                self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], robot, a2)
            else:
                robot.isActive = False

        if not isExisted or self.timeCounter > self.timeLimit:
            # all robot collided -> dead
            self.timeCounter = 0
            self.cycle += 1
            self.numExistRobots = self.population

            self.RunACycle(dt)
            for deadBoy in self.robots:
                deadBoy.Reset()
                for wall in self.gui.walls:
                    deadBoy.updateSensorInfo(deadBoy.pos, wall)

    def updateLogicPerOne(self, dt):
        if self.curIndex < self.population:
            i = self.curIndex
            robot = self.robots[i]
            if not robot.isActive or self.timeCounter > self.timeLimit :
                self.curIndex+=1
                self.numExistRobots = self.population - self.curIndex
                self.timeCounter = 0
                self.isCollided =False
                return



            #copy from gui.updateLogic
            if not self.isCollided:
                # assign to RNNs
                self.a2, a3 = FeedForward(self.rnns[i])
                robot.setVelocity(a3[0], a3[1])
                tempPos = robot.updateTransform(dt)
            else:
                tempPos = np.copy(robot.pos)

            self.isCollided = False

            for wall in self.gui.walls:
                flag = robot.checkCollisionWithNewPos(tempPos, wall)
                if not self.isCollided:
                    self.isCollided = flag

            if not self.isCollided:
                robot.pos = tempPos
            else:
                robot.isActive=False
            if robot.prevPos[0] != robot.pos[0] or robot.prevPos[1] != robot.pos[1]:
                robot.updateSensors()
                robot.sDistances = np.zeros(len(robot.sensors))
                for wall in self.gui.walls:
                    robot.updateSensorInfo(robot.pos, wall)
                self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], robot, self.a2)


            if  robot.pos[0] < 0 - 50 or robot.pos[0] > self.gui.screenWidth + 50 or robot.pos[1] < 0 - 50 or robot.pos[1] > self.gui.screenHeight + 50:
                print("quach thi phung")

        else:
            # all robot collided -> dead
            self.a2 = None
            self.timeCounter = 0
            self.curIndex = 0
            self.cycle += 1
            self.numExistRobots = self.population
            self.isCollided = False
            self.RunACycle(dt)
            for deadBoy in self.robots:
                deadBoy.Reset()
                for wall in self.gui.walls:
                    deadBoy.updateSensorInfo(deadBoy.pos, wall)

    def updateLogic(self, dt):
        dt = dt * self.speedUpAmount
        self.timeCounter +=dt
        #self.updateLogicForAll(dt)
        self.updateLogicPerOne(dt)

    def draw(self, qp):
        #for all
        '''
        for miniRobot in self.robots:
            if miniRobot.isActive:
                miniRobot.draw(qp)
        '''
        #per one
        if  self.curIndex < self.population:
            curRobot =self.robots[self.curIndex]
            if curRobot.isActive:
                curRobot.draw(qp)

        qp.drawText(QPointF(100,50), "Cycle: " + str(self.cycle))
        qp.drawText(QPointF(100,100), "Num existed robots: " + str(self.numExistRobots))
        qp.drawText(QPointF(100,150), "Max fitness: " + str("{:.2f}".format(self.maxFitness)))
        qp.drawText(QPointF(100,200), "Time counter: " + str("{:.2f}".format(self.timeCounter)))




    def RunACycle(self,dt):
        #evaluation
        fitGens = self.fitness(self.robots)
        #selection
        defaultMin = 0

        #get top 3
        maxIndexes = np.zeros(3,dtype=int)
        for i in range(3):
            maxIndexes[i] = np.argmax(fitGens)
            if i==0:
                self.maxFitness = int (fitGens[maxIndexes[i]])
            fitGens[maxIndexes[i]] = defaultMin
        #reproduction
        newGens = np.copy(self.gens)

        #dupCases = int(round(fitGens.shape[0] / 3) )
        for i in range(fitGens.shape[0]):
            index =i% 3
            temp = self.gens[maxIndexes[index]]
            newGens[i,] = temp

        self.gens = newGens

        #crossover
        randomPoint= np.random.randint(low=1,high=self.gens.shape[1]-1)

        dupCases = int(round(self.gens.shape[0] / 2))

        index = 0
        for pair in range (dupCases):
            self.gens[index], self.gens[index + 1] = crossover(randomPoint,self.gens[index],self.gens[index+1])
            index += 2

        #mutation
        normalGen = np.linalg.norm(self.gens)
        for i in range(self.gens.shape[0]):
            for j in range(self.gens.shape[1]):
                ranVal = np.random.rand()
                if ranVal < self.mutationRate:
                    self.gens[i][j] =  np.random.rand() *normalGen




def crossover(randomPoint, gen1,gen2):
    left1 = gen1[0:randomPoint]
    right1 = gen1[randomPoint:]

    left2 = gen2[0:randomPoint]
    right2 = gen2[randomPoint:]

    newGen1 = np.concatenate((left1,right2))
    newGen2 = np.concatenate((left2,right1))

    return newGen1,newGen2