import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json,codecs
from pathlib import Path
import Utils
import statistics

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
    #diversities = np.array(diversities)
    cycleArray = np.arange(0,cycles,dtype=int)

    plt.title("Diversity of population")
    fig, host = plt.subplots()


    par1 = host.twinx()
    #host.plot(cycleArray, maxFitness, 'b-', label="maxFitness")
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

def plotPerformance(filePaths):
    datas = {}
    avgFitness = {}
    maxFitness = {}

    errMaxs = []
    errAvgs = []

    globalCycles = 0
    for path in filePaths:
        p = Path(path)
        if p.is_file():
            obj_text = codecs.open(path, 'r', encoding='utf-8').read()
            data = json.loads(obj_text)

        else:
            raise ("Not found file " + str(path))

        cycles = len(data)
        if globalCycles == 0:
            globalCycles = cycles
            for cyc in range(cycles):
                datas[cyc] = []
                avgFitness[cyc] = []
                maxFitness[cyc] = []

        else:
            if cycles != globalCycles:
                raise ("dataset is inconsistent at " + str(path))

        for cycle in range(cycles):
            generation = data[str(cycle)]
            fitness = generation['fitness']
            temp = np.array(fitness)

            #datas[cycle].append(temp)

            maxFitness[cycle].append(np.max(temp))
            avgFitness[cycle].append(np.average(temp))



        print("loaded file "+str(path)+" - data len =" + str(cycles))

    avgs = []
    maxs = []
    for k in avgFitness:
        temp = np.array(avgFitness[k])
        avg = statistics.mean(temp)

        std = statistics.stdev(temp)
        avgs.append(avg)
        errAvgs.append(std)

    for k in maxFitness:
        temp = np.array(maxFitness[k])
        avg = statistics.mean(temp)#np.mean(temp)

        std = statistics.stdev(temp)#np.std(temp)

        maxs.append(avg)
        errMaxs.append(std)


    cycleArray = np.arange(0,globalCycles,dtype=int)

    #plt.plot(cycleArray,avgs,color='r')
    #plt.plot(cycleArray,maxs,color='g')
    plt.errorbar(cycleArray,avgs,errAvgs,linestyle='-', marker='o',capsize=2,label='Average')

    plt.errorbar(cycleArray,maxs,errMaxs,linestyle=':', marker='^',color='r',capsize=2,label='Max')
    plt.legend()
    plt.xticks(cycleArray)
    plt.savefig('performance.png')
    plt.show()

#plotPerformance(["box_final_2.json","box_final_3.json","box_final_4.json"])

#plotDiversity('double_rect3.json')

#plotDiversity('double_rect4.json')

#plotDiversity('double_rect6.json')