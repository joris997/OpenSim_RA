import sys
import time
import numpy as np
from matplotlib import pyplot as plt
from subprocess import Popen, PIPE


def parse_output(raw_output):
    array = []
    var = 0
    for word in raw_output.decode('utf-8').split('\n'):
        try:
            var = float(word.strip())
        except:
            for char in word.split('\t')[:-2]:
                array.append(float(char.strip()))

    return var, array


def general_loop(variable,loopRange,nTimes,isVec,showVisualizer=False,plotLength=False):
    totalCases = len(loopRange)
    totalRuns = totalCases*nTimes

    runTimes = []
    maxAngles = []
    tests = []
    lengths = []
    for i in range(len(loopRange)):
        instance = loopRange[i]

        if isVec==1:
            args = "./build/ellipsoidTest --" + variable + "=[" + str(instance) + ",0,0]"
        elif isVec==2:
            args = "./build/ellipsoidTest --" + variable + "=[0," + str(instance) + ",0]"
        elif isVec==3:
            args = "./build/ellipsoidTest --" + variable + "=[0,0," + str(instance) + "]"
        else:
            args = "./build/ellipsoidTest --" + variable + "=" + str(instance)

        if showVisualizer:
            args += " --visualizer"

        args += "--cylinder_radius=0.16"
#        args += "--stiffness=1000"

        simTime = 5.0;
        args += " --final_time="+str(simTime)

        tDelta = 0
        for ii in range(0,nTimes):
            p = Popen(args,shell=True,stdout=PIPE)

            t0 = time.time()
            output = p.communicate()[0]
            tDelta += time.time() - t0

            percentage = round((i*nTimes+ii)/totalRuns*100)
            print("[",percentage,"%] Runtime with",str(variable),"=", \
                str(round(instance,5)),":",round(time.time()-t0,5))

            passLengthTest,angles = parse_output(output)

        tests.append(passLengthTest)
        lengths.append(angles)

        runTimes.append(tDelta/nTimes)
        maxAngles.append(max(angles))

    fig,ax1 = plt.subplots()

    color1 = 'tab:red'
    color2 = 'tab:blue'

    ax1.plot(loopRange,tests,color=color1)
    ax1.set_xlabel(str(variable))
    ax1.set_ylabel("pass/fail [bool]",color=color1)
    ax1.tick_params(axis='y',labelcolor=color1)

    if plotLength:
        ax2 = ax1.twinx()
        ax2.plot(loopRange,maxAngles,color=color2)
        ax2.set_ylabel("max angle of tangent [rad]",color=color2)
        ax2.tick_params(axis='y',labelcolor=color2)

    plt.show()

    breakingIndex = int(np.array(tests).tolist().index(0))
    print("Breaking amplitude: " + str(loopRange[breakingIndex]))
    print("Breaking angle:     " + str(maxAngles[breakingIndex]))


def main():
    rotationArray = np.linspace(0.01,0.5,100)
    variable = "amplitude"
    nTimes = 1
    isVec = 0
    general_loop(variable,rotationArray,nTimes,isVec,False,True)


if __name__ == "__main__":
    main()

