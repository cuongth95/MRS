import numpy as np
from Beacon import Beacon
'''
@author Truong Huy Cuong
'''
def genAssignmentMap():
    initRobotPos = [700,400]
    beacons = []
    lines = []
    # beacons pos
    beacons.append(Beacon([0+600, 0+300]))
    beacons.append(Beacon([0+600, 200+300]))
    beacons.append(Beacon([0+600,  200 * 5+300]))

    beacons.append(Beacon([144*2+600,  200*2+300]))
    beacons.append(Beacon([144*2+600,  200*4+300]))


    beacons.append(Beacon([144*4+600,  200 * 3+300]))
    beacons.append(Beacon([144*4+600,  200 * 5+300]))
    beacons.append(Beacon([144*4+600,  200+300]))

    beacons.append(Beacon([144*6+600, 0 + 300]))
    beacons.append(Beacon([144*6+600, 200*2+300]))
    beacons.append(Beacon([144*6+600, 200 *5 + 300]))


    beacons.append(Beacon([144*2+600, 200+300]))
    beacons.append(Beacon([144*6+600, 200+300]))
    beacons.append(Beacon([0+600,  200 * 3+300]))
    beacons.append(Beacon([144*4+600, 200*2+300]))
    #LINES
    lines.append([1,2])
    lines.append([3,2])
    lines.append([3,7])
    lines.append([11,7])
    lines.append([11,10])
    lines.append([10,9])
    lines.append([1,9])
    lines.append([2,8])
    lines.append([4,10])
    lines.append([4,5])
    lines.append([6,7])
    return initRobotPos, beacons,lines

def genRandomDenseMap():
    initRobotPos = [700, 400]
    beacons = []
    lines = []
    for i in range(150):
        tempX = 200 + np.random.rand()  * (1800 - 200)
        tempY = 200 + np.random.rand()  * (900 - 200)
        #tempY = np.random.normal(400,50)
        beacons.append(Beacon([tempX,tempY]))

    return initRobotPos,beacons,lines