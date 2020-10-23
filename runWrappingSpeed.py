import sys
import time
import numpy as np
from matplotlib import pyplot as plt
from subprocess import Popen, PIPE


def parse_output(raw_output):
    for word in raw_output.decode('utf-8').split('\n'):
        try:
            var = float(word.strip())
        except:
            springLength = []
            for char in word.split('\t')[:-2]:
                springLength.append(float(char.strip()))

    # just assume that the last list of output is the spring length
    return springLength


def general_loop(variable,loopRange,nTimes,isVec,showVisualizer=False):
    totalCases = len(loopRange)
    totalRuns = totalCases*nTimes

    runTimes = []
    for i in range(len(loopRange)):
        instance = loopRange[i]

        if isVec==1:
            args = "./build/wrappingSpeed --" + variable + "=[" + str(instance) + ",0,0]"
        elif isVec==2:
            args = "./build/wrappingSpeed --" + variable + "=[0," + str(instance) + ",0]"
        elif isVec==3:
            args = "./build/wrappingSpeed --" + variable + "=[0,0," + str(instance) + "]"
        else:
            args = "./build/wrappingSpeed --" + variable + "=" + str(instance)

        if showVisualizer:
            args += " --visualizer"

        tDelta = 0
        for ii in range(0,nTimes):
            p = Popen(args,shell=True,stdout=PIPE)

            t0 = time.time()
            output = p.communicate()[0]
            tDelta += time.time() - t0

            percentage = round((i*nTimes+ii)/totalRuns*100)
            print("[",percentage,"%] Runtime with",str(variable),"=",str(round(instance,5)),":",round(time.time()-t0,5))

            springLength = parse_output(output)

        runTimes.append(tDelta/nTimes)

    plt.plot(loopRange,runTimes)
    plt.xlabel(str(variable))
    plt.ylabel("runtime [s]")
    plt.show()


def main():
    rotationArray = np.linspace(0,1,10)
#    rotationArray = np.linspace(10,5000,250)
    variable = "cylinder_rotation"
    nTimes = 10
    isVec = 2
    general_loop(variable,rotationArray,nTimes,isVec)

#    isVec = 1
#    general_loop(variable,rotationArray,nTimes,isVec)
#    isVec = 2
#    general_loop(variable,rotationArray,nTimes,isVec)
#    isVec = 3
#    general_loop(variable,rotationArray,nTimes,isVec)


if __name__ == "__main__":
    main()
