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

    runTimesCyl = np.zeros(len(loopRange))
    runTimesEll = np.zeros(len(loopRange))
    runTimesSph = np.zeros(len(loopRange))
    runTimesBlk = np.zeros(len(loopRange))
    wrappingSurfaces = {"cylinder","ellipsoid"}

    for surface in wrappingSurfaces:
        for i in range(nTimes):
            for ii in range(len(loopRange)):
                instance = loopRange[ii]

                executable = "wrappingSpeed"       # falling masses
#                executable = "wrappingSurfaces"    # forcing function
                if isVec==1:
                    args = "./build/" + executable + " --" + variable + "=[" + str(instance) + ",0,0]"
                elif isVec==2:
                    args = "./build/" + executable + " --" + variable + "=[0," + str(instance) + ",0]"
                elif isVec==3:
                    args = "./build/" + executable + " --" + variable + "=[0,0," + str(instance) + "]"
                else:
                    args = "./build/" + executable + " --" + variable + "=" + str(instance)

                if showVisualizer:
                    args += " --visualizer"

                simTime = 10.0;
                args += " --final_time="+str(simTime)
                args += " --type="+str(surface)
                print(args)

                p = Popen(args,shell=True,stdout=PIPE)

                t0 = time.time()
                output = p.communicate()[0]
                runTime = time.time()-t0

                if surface == "cylinder":
                    runTimesCyl[ii] += runTime
                elif surface == "ellipsoid":
                    runTimesEll[ii] += runTime
                elif surface == "sphere":
                    runTimesSph[ii] += runTime
                else:
                    runTimesBlk[ii] += runTime

                percentage = round((i*len(loopRange)+ii)/totalRuns*100)
                print("[",percentage,"%] Runtime with",str(variable),"=", \
                    str(round(instance,5)),":",round(time.time()-t0,5))

    runTimesCyl = runTimesCyl/nTimes
    runTimesEll = runTimesEll/nTimes
    runTimesSph = runTimesSph/nTimes
    runTimesBlk = runTimesBlk/nTimes

    fig,ax1 = plt.subplots()

    color1 = 'tab:red'
    color2 = 'tab:blue'

    ax1.plot(loopRange,runTimesCyl,color=color1)
    ax1.plot(loopRange,runTimesEll,color=color2)
    ax1.set_xlabel(str(variable))
    ax1.set_ylabel("runtime [s]")
    ax1.legend(['cylinder','ellipsoid'])

    plt.show()


def main():
#    variableArray = np.linspace(0.1,0.45,100)
#    variable = "cylinder_radius"
#    nTimes = 30
#    isVec = 0

    variableArray = np.linspace(0,1,100)
    variable = "cylinder_rotation"
    nTimes = 20
    isVec = 2

    general_loop(variable,variableArray,nTimes,isVec)


if __name__ == "__main__":
    main()

