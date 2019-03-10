from hmac import new
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import json,codecs
from Robot import Robot
from collections import namedtuple
import numpy as np
from pathlib import Path
'''
@author Truong Huy Cuong
'''
def sigmoid(z):
    ret = 1.0 / (1.0 + np.exp(-z))
    return ret
def softPlus(z):
    #ret = - np.log(1.0 - 1.0/(1.0 + np.exp(-z)))
    #ret = np.log(1 + np.exp(z))
    ret = np.maximum(z,0)
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

    def __init__(self,gui,startPos,sThreshold,filePath):


        self.gui = gui
        #self.mainRobot = gui.robot
        self.mutationRate = 0.1
        self.startPos = startPos
        self.filePath = filePath
        self.Initilization(sThreshold)


    def Initilization(self,threshold,population=20):
        self.offset = 10
        self.maxRow = int(self.gui.screenWidth/self.offset)
        self.maxCol =int(self.gui.screenHeight/self.offset)
        self.checkPoints = np.ones((self.maxRow,self.maxCol))


        self.curIndex=0
        self.population = population
        self.numExistRobots = population
        self.maxFitness = 0
        self.cycle =0
        hiddenNodes = 4
        self.maxRange = threshold
        #4 + (4 * (12 + 4)) + 2 + (2 * 4)#6+(6*(12+6))+2+(2*6)
        self.gens = np.random.rand(population,4 + (4 * (12 + 4)) + 2 + (2 * 4)) *0.1 #np.random.randint(self.maxRange,size = (population,4+64+2+8)) #
        # self.gens = np.concatenate((self.gens, np.zeros([hiddenNodes])))
        self.robots = []
        self.rnns = []
        self.a2s = []
        #self.prevPoses = []
        for i in range(population):
            temp = Robot(assignId=False,startPos= self.startPos)
            temp.updateSensors()
            temp.sDistances = np.full(len(temp.sensors), temp.sThreshold)
            for wall in self.gui.walls:
                temp.updateSensorInfo(temp.pos, wall)
            #self.gui.checkCollision(temp) #update sensors
            self.robots.append(temp)
            hid = np.zeros(4)
            tempRNN = self.GetRNNParamsByGen(self.gens[i],temp,hid)
            self.a2s.append(0)
            #self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], temp, a2)
            self.rnns.append(tempRNN)#RNNInit(inp,hid))
            #self.gens.append(np.copy(temp.sDistances))
            #self.prevPoses.append(np.copy(temp.pos))



    def fitness(self,r):
        f = np.zeros((len(r), 1))



        for i, robot in enumerate(r):

            if robot.score > 0:
                f[i] =  robot.score  # self.objectiveDistanceNorm(dist)
            else:
                f[i] = 0



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



    speedUpAmount = 1
    timeCounter = 0
    timeLimit = 2 *speedUpAmount
    isCollided = False

    def updateLogicForAll(self, dt):
        isExisted = False
        self.numExistRobots = self.population
        for i, robot in enumerate(self.robots):
            if not robot.isActive:
                self.numExistRobots -=1
                continue
            isExisted=True
            # copy from gui.updateLogic
            if not self.isCollided:
                # assign to RNNs
                self.a2s[i], a3 = FeedForward(self.rnns[i])
                robot.setVelocity(a3[0], a3[1])
                tempPos = robot.updateTransform(dt)
            else:
                tempPos = np.copy(robot.pos)

            self.isCollided = False

            for wall in self.gui.walls:
                flag = robot.checkCollisionWithNewPos(tempPos, wall, self.isCollided)
                if not self.isCollided:
                    self.isCollided = flag

            if not self.isCollided:
                robot.pos = tempPos

            else:
                # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
                # robot.score = robot.score/3
                #if robot.isActive:  # punishment
                #    robot.score = robot.score / 2
                #robot.score -= 0.5
                robot.isActive = False

            if robot.prevPos[0] != robot.pos[0] or robot.prevPos[1] != robot.pos[1]:

                # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
                '''
                if robot.isActive:
                    row = int(robot.pos[0]/self.offset)
                    col = int(robot.pos[1]/self.offset)
                    if row>=0 and row <self.maxRow and col >=0 and col < self.maxCol:
                        if self.checkPoints[row][col] == 1:

                            robot.score += 1
                            self.checkPoints[row][col] = 0
                '''
                row = int(robot.pos[0] / self.offset)
                col = int(robot.pos[1] / self.offset)
                if row >= 0 and row < self.maxRow and col >= 0 and col < self.maxCol:
                    if self.checkPoints[row][col] == 1:
                        if robot.isActive:
                            robot.score += 1
                        # else:
                        # robot.score -= 1
                        self.checkPoints[row][col] = 0
                '''
                row = int(robot.pos[0] / self.offset)
                col = int(robot.pos[1] / self.offset)
                if row >= 0 and row < self.maxRow and col >= 0 and col < self.maxCol:
                    if self.checkPoints[row][col] == 1:
                        self.checkPoints[row][col] = 0
                '''

                robot.updateSensors()
                robot.sDistances = np.full(len(robot.sensors), robot.sThreshold)  # np.zeros(len(robot.sensors))
                for wall in self.gui.walls:
                    robot.updateSensorInfo(robot.pos, wall)
                self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], robot, self.a2s[i])

            if robot.pos[0] < 0 - 50 or robot.pos[0] > self.gui.screenWidth + 50 or robot.pos[1] < 0 - 50 or robot.pos[
                1] > self.gui.screenHeight + 50:
                print("quach thi phung")

        if not isExisted: #or self.timeCounter > self.timeLimit:
            # all robot collided -> dead
            self.timeCounter = 0
            self.cycle += 1
            self.numExistRobots = self.population

            self.RunACycle(dt)
            for deadBoy in self.robots:
                deadBoy.Reset(startPos=self.startPos)
                for wall in self.gui.walls:
                    deadBoy.updateSensorInfo(deadBoy.pos, wall)

    def updateLogicWithSpecificGenome(self,i,dt):
        robot = self.robots[i]
        if not self.isCollided:
            self.a2, a3 = FeedForward(self.rnns[i])
            robot.setVelocity(a3[0], a3[1])
            tempPos = robot.updateTransform(dt)
        else:
            tempPos = np.copy(robot.pos)

            # tempPos = np.array([1056.009,500])
        isCollided = False
        for wall in self.gui.walls:
            flag = robot.checkCollisionWithNewPos(tempPos, wall, isCollided)
            if not isCollided:
                isCollided = flag

        if not isCollided:
            robot.pos = tempPos
            row = int(robot.pos[0] / self.offset)
            col = int(robot.pos[1] / self.offset)
            if row >= 0 and row < self.maxRow and col >= 0 and col < self.maxCol:
                if self.checkPoints[row][col] == 1:
                    if robot.isActive:
                        robot.score += 1
        else:
            print("ROBOT COLLIDED!!!!")
            print("robot fitness = "+str(robot.score))
            sys.exit()

        if robot.prevPos[0] != robot.pos[0] or robot.prevPos[1] != robot.pos[1]:
            robot.updateSensors()
            robot.sDistances = np.full(len(robot.sensors),
                                            robot.sThreshold)  # np.zeros(len(self.robot.sensors))
            for wall in self.gui.walls:
                robot.updateSensorInfo(robot.pos, wall)
            self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], robot, self.a2)
        print('pos='+str(robot.pos))

    def updateLogicPerOne(self, dt):
        if self.curIndex < self.population:
            i = self.curIndex
            robot = self.robots[i]

            # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
            #if not robot.isActive:# or self.timeCounter > self.timeLimit :
            if  self.timeCounter >= self.timeLimit :
            #if robot.score <=0 or self.timeCounter > self.timeLimit :
                self.checkPoints = np.ones((self.maxRow, self.maxCol))
                self.curIndex+=1
                self.numExistRobots = self.population - self.curIndex
                self.timeCounter = 0
                self.isCollided =False
                self.a2 = None
                print("robot ("+str(i+1)+"/"+str(self.population)+")-score= "+str(robot.score))
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
                flag = robot.checkCollisionWithNewPos(tempPos, wall,self.isCollided)
                if not self.isCollided:
                    self.isCollided = flag

            if not self.isCollided:
                robot.pos = tempPos

            else:
                # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
                #robot.score = robot.score/3
                #if robot.isActive: #punishment
                #   robot.score = robot.score / 2
                #robot.score -= 0.5
                robot.isActive=False

            if not robot.isActive:
                self.timeCounter =   self.timeLimit#+= dt
            #if not robot.isActive:

            if robot.prevPos[0] != robot.pos[0] or robot.prevPos[1] != robot.pos[1]:

                # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
                '''
                if robot.isActive:
                    row = int(robot.pos[0]/self.offset)
                    col = int(robot.pos[1]/self.offset)
                    if row>=0 and row <self.maxRow and col >=0 and col < self.maxCol:
                        if self.checkPoints[row][col] == 1:
                            
                            robot.score += 1
                            self.checkPoints[row][col] = 0
                '''
                row = int(robot.pos[0] / self.offset)
                col = int(robot.pos[1] / self.offset)
                if row >= 0 and row < self.maxRow and col >= 0 and col < self.maxCol:
                    if self.checkPoints[row][col] == 1:
                        if robot.isActive:
                            robot.score += 1
                        #else:
                            #robot.score -= 1
                        self.checkPoints[row][col] = 0
                '''
                row = int(robot.pos[0] / self.offset)
                col = int(robot.pos[1] / self.offset)
                if row >= 0 and row < self.maxRow and col >= 0 and col < self.maxCol:
                    if self.checkPoints[row][col] == 1:
                        self.checkPoints[row][col] = 0
                '''


                robot.updateSensors()
                robot.sDistances = np.full(len(robot.sensors),robot.sThreshold) #np.zeros(len(robot.sensors))
                for wall in self.gui.walls:
                    robot.updateSensorInfo(robot.pos, wall)
                self.rnns[i] = self.GetRNNParamsByGen(self.gens[i], robot, self.a2)


            if  robot.pos[0] < 0 - 50 or robot.pos[0] > self.gui.screenWidth + 50 or robot.pos[1] < 0 - 50 or robot.pos[1] > self.gui.screenHeight + 50:
                print("quach thi phung")

        else:
            # all robot collided -> dead
            self.checkPoints = np.ones((self.maxRow, self.maxCol))
            self.a2 = None
            self.timeCounter = 0
            self.curIndex = 0
            self.saveFile()
            self.cycle += 1
            self.numExistRobots = self.population
            self.isCollided = False
            self.RunACycle(dt)
            for deadBoy in self.robots:
                deadBoy.Reset(startPos= self.startPos)
                for wall in self.gui.walls:
                    deadBoy.updateSensorInfo(deadBoy.pos, wall)

    def saveFile(self):
        #print(str(self.gens))
        acycle= {}
        acycle['fitness']= self.fitness(self.robots).tolist()
        acycle['genomes']= self.gens.tolist()
        p = Path(self.filePath)
        if p.is_file():
            obj_text = codecs.open(self.filePath, 'r', encoding='utf-8').read()
            data = json.loads(obj_text)
        else:
            data = {}
        data[str(self.cycle)]= acycle
        with codecs.open(self.filePath, 'w', encoding='utf-8') as file:
            json.dump(data, file, separators=(',', ':'), sort_keys=True, indent=4)
        ''''
        with open(self.filePath , 'w') as file:
            for i in range(self.population):
                for j in range(len(self.gens[i])):
                    file.write(str("{:.8f} ".format(float(self.gens[i,j]))))
                file.write("\n")
        '''

    def loadFile(self):
        obj_text = codecs.open(self.filePath, 'r', encoding='utf-8').read()
        data = json.loads(obj_text)
        print("loaded - data len ="+str(len(data)))
        '''
        genomes = []
        with open(self.genomesFilePath , 'r') as file:
            for i in range(20):
                temp = file.readline()
                genomes.append(np.fromstring(temp, dtype=float, sep=' '))

        self.gens = np.copy(genomes)
        for i in range(20):
            self.rnns[i]= self.GetRNNParamsByGen(self.gens[i], self.robots[i],np.zeros(4))
        print("done")
        '''

    def updateLogic(self, dt):
        dt = dt * self.speedUpAmount
        # GOOD FOR SEARCHING NEW AREA, BUT BAD FOR AVOIDING COLLISION
        #self.timeCounter +=dt
        #self.updateLogicForAll(dt)
        self.updateLogicPerOne(dt)

    def drawSpecificGenome(self,i,qp):
        self.robots[i].draw(qp)

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
            #if curRobot.isActive:
            curRobot.draw(qp)
            qp.drawText(QPointF(100,300), "curr fitness: " + str("{:.2f}".format(curRobot.score)))

        pen = QPen(Qt.red, 1, Qt.SolidLine)
        qp.setPen(pen)
        for i in range(self.maxRow):
            for j in range(self.maxCol):
                if self.checkPoints[i][j] == 1:
                    qp.drawRect(i*self.offset,j*self.offset,2,2)


        qp.drawText(QPointF(100,50), "Cycle: " + str(self.cycle))
        qp.drawText(QPointF(100,100), "Num existed robots: " + str(self.numExistRobots))
        qp.drawText(QPointF(100,150), "Max fitness: " + str("{:.2f}".format(self.maxFitness)))
        qp.drawText(QPointF(100,200), "Time counter: " + str("{:.2f}".format(self.timeCounter)))




    def RunACycle(self,dt):
        print("cycle = "+str(self.cycle-1))
        #evaluation
        fitGens = self.fitness(self.robots)
        #selection
        defaultMin = 0
        for i in range(len(fitGens)):
            print("f["+str(i)+"]="+str(fitGens[i]))
        #get top 3
        maxIndexes = np.zeros(3,dtype=int)
        for i in range(3):
            maxIndexes[i] = np.argmax(fitGens)
            if i==0:
                self.maxFitness = float(fitGens[maxIndexes[i]])
            fitGens[maxIndexes[i]] = defaultMin

        print("max fitness = "+str(self.maxFitness ))
        #reproduction
        newGens = np.copy(self.gens)

        firstRank = int(self.population / 2)+1
        secondRank = int(self.population/3)
        thirdRank = self.population - firstRank - secondRank

        for i in range(0,firstRank):
            temp = self.gens[maxIndexes[0]]
            newGens[i,] = temp

        for i in range(firstRank,firstRank + secondRank):
            temp = self.gens[maxIndexes[1]]
            newGens[i,] = temp

        for i in range(firstRank + secondRank,firstRank + secondRank+thirdRank):
            temp = self.gens[maxIndexes[2]]
            newGens[i,] = temp

        self.gens = newGens

        #crossover
        randomPoint= np.random.randint(low=1,high=self.gens.shape[1]-1)

        dupCases = int(round(self.gens.shape[0] / 2))

        index = self.population-1
        for pair in range (dupCases):
            self.gens[index], self.gens[index - 1] = crossover(randomPoint,self.gens[index],self.gens[index-1])
            index -= 2

        #mutation
        minGen = 0.002 #0 + np.random.rand() *(0.002-0)
        maxGen = 0.15#minGen + np.random.rand() *(0.3-minGen)
        mutationAmount = 0.09
        print("min="+str(minGen)+",max="+str(maxGen))
        for i in range(self.gens.shape[0]):
            for j in range(self.gens.shape[1]):
                if np.random.rand() < self.mutationRate:
                    self.gens[i][j] = minGen + np.random.rand() *(maxGen-minGen)  # minGen +  np.random.rand() *(mutationAmount-minGen)




def crossover(randomPoint, gen1,gen2):
    left1 = gen1[0:randomPoint]
    right1 = gen1[randomPoint:]

    left2 = gen2[0:randomPoint]
    right2 = gen2[randomPoint:]

    newGen1 = np.concatenate((left1,right2))
    newGen2 = np.concatenate((left2,right1))

    return newGen1,newGen2