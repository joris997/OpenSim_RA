import sys
import time
import numpy as np
from matplotlib import pyplot as plt
from subprocess import Popen, PIPE


def parse_output(raw_output):
    springLength = []
    var = 0
    for word in raw_output.decode('utf-8').split('\n'):
        try:
            var = float(word.strip())
        except:
            for char in word.split('\t')[:-2]:
                springLength.append(float(char.strip()))

    return var


def general_loop(variable,loopRange,nTimes,isVec,showVisualizer=False,plotLength=False):
    totalCases = len(loopRange)
    totalRuns = totalCases*nTimes

    runTimes = []
    maxSpringLength = []
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

        simTime = 5.0;
        args += " --final_time="+str(simTime)

        tDelta = 0
        maxLength = 0
        for ii in range(0,nTimes):
            p = Popen(args,shell=True,stdout=PIPE)

            t0 = time.time()
            output = p.communicate()[0]
            tDelta += time.time() - t0

            percentage = round((i*nTimes+ii)/totalRuns*100)
            print("[",percentage,"%] Runtime with",str(variable),"=", \
                str(round(instance,5)),":",round(time.time()-t0,5))

            maxLength += max(parse_output(output))

        runTimes.append(tDelta/nTimes)
        maxSpringLength.append(maxLength/nTimes)

    fig,ax1 = plt.subplots()

    color1 = 'tab:red'
    color2 = 'tab:blue'

    ax1.plot(loopRange,runTimes,color=color1)
    ax1.set_xlabel(str(variable))
    ax1.set_ylabel("runtime [s]",color=color1)
    ax1.tick_params(axis='y',labelcolor=color1)

    if plotLength:
        ax2 = ax1.twinx()
        ax2.plot(loopRange,maxSpringLength,color=color2)
        ax2.set_ylabel("max spring length [m]",color=color2)
        ax2.tick_params(axis='y',labelcolor=color2)

    plt.show()


def main():
    rotationArray = np.linspace(0.01,1.00,100)
    variable = "amplitude"
    nTimes = 10
    isVec = 0
    general_loop(variable,rotationArray,nTimes,isVec)


if __name__ == "__main__":
    main()
