import numpy as np

def normalize(vec):
    # get max and min
    maxVec = max(vec)
    minVec = min(vec)

    if maxVec == minVec:
        return np.zeros(vec.shape)
    # normalize to -1...1
    #1.0 / sum(vec) * vec  # (
    vecN = (vec-minVec) /(maxVec-minVec) #- 0.5 ) *2;
    #vecN = ((vec - minVec)/ (maxVec - minVec) - 0.5) * 2
    return vecN