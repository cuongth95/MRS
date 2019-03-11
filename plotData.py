import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json,codecs
from pathlib import Path
import Utils


def plotDiversity(filePath):
    p = Path(filePath)
    if p.is_file():
        obj_text = codecs.open(filePath, 'r', encoding='utf-8').read()
        data = json.loads(obj_text)
    else:
        raise ("Not found file " + str(filePath))

    cycles = len(data)
    print("loaded - data len =" + str(cycles))

    maxFitness = []
    diversities = []

    for cycle in range(cycles):
        generation = data[str(cycle)]
        fitness = generation['fitness']
        temp = np.array(fitness)
        maxFitness.append(np.max(temp))
        genomes = generation['genomes']
        temp = np.array(genomes)
        diversities.append( Utils.diversityFunction(temp))


    maxFitness = np.array(maxFitness)
    diversities = np.array(diversities)
    cycleArray = np.arange(0,cycles,dtype=int)

    plt.title("Diversity of population")
    fig, host = plt.subplots()


    par1 = host.twinx()
    host.plot(cycleArray,diversities,'b-',label="diversity")
    par1.plot(cycleArray,maxFitness,'r-',label="max fitness")

    plt.xticks(cycleArray)

    host.set_xlim(0, cycles)
    host.set_ylim(bottom=0)
    par1.set_ylim(bottom=0)

    host.set_xlabel('Generations')
    host.set_ylabel('Diversity',color='b')
    par1.set_ylabel('Max fitness',color='r',rotation=-90)
    host.get_yaxis().set_label_coords(-0.115, 0.5)
    par1.get_yaxis().set_label_coords(1.11, 0.5)
    #plt.plot(cycleArray,diversities,'r',cycleArray,maxFitness,'b')

    plt.savefig(filePath+'_diversity.png')
    plt.show()

#plotDiversity('box_weights3.json')

