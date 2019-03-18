import numpy as np
import json
'''
@author Truong Huy Cuong
'''
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 720
BLACK = (0,0,0)
def makeRect(origin, rsize):
    pos = np.array([origin[0], SCREEN_HEIGHT - origin[1]])
    tl = np.array([pos[0] - rsize[0] / 2, pos[1] - rsize[1] / 2])
    tr = np.array([pos[0] + rsize[0] / 2, pos[1] - rsize[1] / 2])
    bl = np.array([pos[0] - rsize[0] / 2, pos[1] + rsize[1] / 2])
    br = np.array([pos[0] + rsize[0] / 2, pos[1] + rsize[1] / 2])

    verts = [tl[0], tl[1],
             tr[0], tr[1],
             br[0], br[1],
             bl[0], bl[1]]
    return verts

def makeCircle(origin,rad,sides=100):
    py_pos = np.array([origin[0], SCREEN_HEIGHT - origin[1]])
    verts = []
    for i in range(100):
        angle= i * 2* np.pi/sides
        #angle = np.rad2deg (float(i) / numPoints * 360.0)
        x = rad[0]/2 * np.cos(angle) + py_pos[0]
        y = rad[1]/2 * np.sin(angle) + py_pos[1]
        verts += [x, y]

    return verts

def makeLine(p1,p2):
    py_pos1= np.array([p1[0],SCREEN_HEIGHT-p1[1]])
    py_pos2= np.array([p2[0],SCREEN_HEIGHT-p2[1]])
    verts = [py_pos1[0], py_pos1[1],
             py_pos2[0], py_pos2[1]]
    return verts

def vec2pgVec(p):
    py_pos = [p[0],SCREEN_HEIGHT-p[1]]
    return py_pos

def normalizeByDivideNorm(vec):
    norm = np.linalg.norm(vec)
    vecN = np.copy(vec)

    if norm != 0:
        vecN[0] /= norm
        vecN[1] /= norm
        return vecN
    else:
        return vec

def normalize(vec):
    vecN = np.array([0,0])
    if vec[0] <0:
        vecN[0] = - 1
    elif vec[0] >0:
        vecN[0] = 1
    else:
        vecN[0] = 0

    #if vecN[0] != 0:
    #   vecN[1] =0
    #    return vecN

    if vec[1] <0:
        vecN[1] = - 1
    elif vec[1] >0:
        vecN[1] = 1
    else:
        vecN[1] = 0
    return vecN

def diversityFunction(g):
    r = g.ravel()
    y = []
    for i in range(0, r.shape[0] - 1):
        for j in range(i + 1, r.shape[0]):
            y.append([r[i], r[j]])
    y = np.array(y)

    d =  (y[:,1] - y[:,0])**2
    ret = np.sum(d)
    return ret