import numpy as np

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