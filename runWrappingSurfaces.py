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


def general_loop(variable,loopRange,nTimes,isVec,showVisualizer=False):
    totalCases = len(loopRange)
    totalRuns = totalCases*nTimes

    runTimesCyl = [0]*len(loopRange)
    runTimesEll = [0]*len(loopRange)
    runTimesSph = [0]*len(loopRange)
    runTimesBlk = [0]*len(loopRange)
    wrappingSurfaces = {"cylinder","ellipsoid"}

    for surface in wrappingSurfaces:
        for i in range(nTimes):
            for ii in range(len(loopRange)):
                instance = loopRange[ii]
                if isVec==1:
                    args = "./build/wrappingSurfaces --" + variable + "=[" + str(instance) + ",0,0]"
                elif isVec==2:
                    args = "./build/wrappingSurfaces --" + variable + "=[0," + str(instance) + ",0]"
                elif isVec==3:
                    args = "./build/wrappingSurfaces --" + variable + "=[0,0," + str(instance) + "]"
                else:
                    args = "./build/wrappingSurfaces --" + variable + "=" + str(instance)

                if showVisualizer:
                    args += " --visualizer"

                simTime = 5.0;
                args += " --final_time="+str(simTime)

                p = Popen(args,shell=True,stdout=PIPE)

                t0 = time.time()
                output = p.communicate()[0]
                if surface == "cylinder":
                    runTimesCyl[ii] += time.time() - t0
                elif surface == "ellipse":
                    runTimesEll[ii] += time.time() - t0
                elif surface == "sphere":
                    runTimesSph[ii] += time.time() - t0
                else:
                    runTimesBlk[ii] += time.time() - t0

                percentage = round((i*len(loopRange)+ii)/totalRuns*100)
                print("[",percentage,"%] Runtime with",str(variable),"=", \
                    str(round(instance,5)),":",round(time.time()-t0,5))

    runTimesCyl = runTimesCyl/nTimes

    fig,ax1 = plt.subplots()

    color1 = 'tab:red'
    color2 = 'tab:blue'

    ax1.plot(loopRange,runTimesCyl,color=color1)
    ax1.plot(loopRange,runTimesEll,color=color1)
    ax1.plot(loopRange,runTimesSph,color=color1)
    ax1.plot(loopRange,runTimesBlk,color=color1)
    ax1.set_xlabel(str(variable))
    ax1.set_ylabel("runtime [s]",color=color1)
    ax1.tick_params(axis='y',labelcolor=color1)

    plt.show()


def main():
    variableArray = np.linspace(0.5,2,100)
    variable = "cylinder_radius"
    nTimes = 1
    isVec = 0
    general_loop(variable,variableArray,nTimes,isVec,False)


if __name__ == "__main__":
    main()

