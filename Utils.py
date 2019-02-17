import numpy as np

def normalize(vec):
    # get max and min
    maxVec = max(vec);
    minVec = min(vec);

    # normalize to -1...1
    vecN = ((vec-minVec) /(maxVec-minVec) - 0.5 ) *2;
    return vecN