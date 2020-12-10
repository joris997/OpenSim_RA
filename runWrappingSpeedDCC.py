#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import numpy as np
from matplotlib import pyplot as plt
from subprocess import Popen, PIPE


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

        simTime = 20.0;
        args += " --final_time="+str(simTime)
#        args += " --cylinder_rotation=[0,0.8,0]"

        tDelta = 0
        maxLength = 0
        for ii in range(0,nTimes):
            p = Popen(args,shell=True,stdout=PIPE)

            t0 = time.time()
            output = p.communicate()[0]
            tDelta += time.time() - t0

            percentage = round((i*nTimes+ii)/totalRuns*100)
            try:
                print("[",percentage,"%] Runtime with",str(variable),"=", \
                    str(round(instance,5)),":",round(time.time()-t0,5))
            except:
                print("[",percentage,"%] Runtime with",str(variable),"=", \
                    str(instance),":",round(time.time()-t0,5))

        runTimes.append(tDelta/nTimes)

    fig,ax1 = plt.subplots()

    color1 = 'tab:red'

    ax1.plot(loopRange,runTimes,color=color1)
    ax1.set_xlabel(str(variable))
    ax1.set_ylabel("runtime [s]",color=color1)
    ax1.tick_params(axis='y',labelcolor=color1)

    plt.show()


def main():
#    rotationArray = np.linspace(1,1.2,10)
#    variable = "body_mass_factor"
#    rotationArray = np.linspace(0,1,20)
    rotationArray = ["ellipsoid","cylinder","viapoints"]
    variable = "type"
    nTimes = 100
    isVec = 0
    general_loop(variable,rotationArray,nTimes,isVec)


if __name__ == "__main__":
    main()
