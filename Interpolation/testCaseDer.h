#include <vector>
#include <iostream>
#include <chrono>
#include "Interpolate.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using std::vector;
using std::pair;
using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::GeometryPath;

void testCase3DDer(){
    // 3D EXAMPLE
    std::cout << "\n" << "3D derivative test-case" << std::endl;
    // Create the interpolation field
    vector<double> xRange;
    vector<double> yRange;
    vector<double> zRange;
    linspace(xRange,0,10,41);
    linspace(yRange,0,10,15);
    linspace(zRange,0,10,18);

    vector<vector<double>> discretization;
    discretization.push_back(xRange);
    discretization.push_back(yRange);
    discretization.push_back(zRange);

    // Create evaluations of wrapping length functions
    vector<pair<vector<int>,double>> evalsPair;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            for (int iii=0; iii<zRange.size(); iii++){
                vector<int> newIndex;
                newIndex.push_back(i); newIndex.push_back(ii); newIndex.push_back(iii);
                pair<vector<int>,double> newPair;
                newPair.first = newIndex;
                newPair.second = xRange[i]*yRange[ii]*zRange[iii];
                evalsPair.push_back(newPair);
            }
        }
    }

    // Create an interpolation object
    Interpolate a = Interpolate(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.61,6.5123,2);
    std::chrono::nanoseconds interp_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        for (int ii=0; ii<xRangeI.size(); ii++){
            for (int iii=0; iii<xRangeI.size(); iii++){
                x.clear();
                x.push_back(xRangeI[i]); x.push_back(xRangeI[ii]); x.push_back(xRangeI[iii]);
                auto before = std::chrono::high_resolution_clock::now();
                ans = a.getInterpDer(x,0);
                auto after = std::chrono::high_resolution_clock::now();
                auto dt = after - before;
                interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
                ++count;
                std::cout << "\n vals: " << x[0] <<" "<< x[1] <<" "<<x[2] << std::endl;
                std::cout << "I1: " << a.getInterpDer(x,0) << std::endl;
//                std::cout << "I2: " << a.getInterpDerFunc(x,0) << std::endl;
                std::cout << "R: " << xRangeI[ii]*xRangeI[iii] << std::endl;
            }
        }
    }
    interp_time /= count;
    std::cout << "interp nanos = " << interp_time.count() << std::endl;
}


void testCase2DDer(){
    // 2D EXAMPLE
    std::cout << "\n" << "2D derivative test-case" << std::endl;
    // Create the interpolation field
    vector<double> xRange;
    vector<double> yRange;
    linspace(xRange,0,10,121);
    linspace(yRange,0,10,141);

    vector<vector<double>> discretization;
    discretization.push_back(xRange);
    discretization.push_back(yRange);

    // Create evaluations of wrapping length functions
    vector<pair<vector<int>,double>> evalsPair;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            vector<int> newIndex;
            newIndex.push_back(i); newIndex.push_back(ii);
            pair<vector<int>,double> newPair;
            newPair.first = newIndex;
            newPair.second = xRange[i]*yRange[ii];
            evalsPair.push_back(newPair);
        }
    }

    // Create an interpolation object
    Interpolate a = Interpolate(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.5123,9.5123,5);
    std::chrono::nanoseconds interp_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        for (int ii=0; ii<xRangeI.size(); ii++){
            x.clear();
            x.push_back(xRangeI[i]); x.push_back(xRangeI[ii]);
            auto before = std::chrono::high_resolution_clock::now();
            ans = a.getInterpDer(x,0);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
            ++count;

            std::cout << "\nI1: " << a.getInterpDer(x,0) << std::endl;
            std::cout << "I2: " << a.getInterpDerFunc(x,0) << std::endl;
            std::cout << "R: " << xRangeI[ii] << std::endl;
        }
    }
    interp_time /= count;
    std::cout << "interp nanos = " << interp_time.count() << std::endl;
}


void testCase1DDer(){
    std::cout << "\n" << "1D derivative test-case" << std::endl;
    // Create the interpolation field
    vector<double> xRange;
    linspace(xRange,0,10,121);

    vector<vector<double>> discretization;
    discretization.push_back(xRange);

    // Create evaluations of wrapping length functions
    vector<pair<vector<int>,double>> evalsPair;
    for (int i=0; i<xRange.size(); i++){
        vector<int> newIndex;
        newIndex.push_back(i);
        pair<vector<int>,double> newPair;
        newPair.first = newIndex;
        newPair.second = xRange[i]*xRange[i];
        evalsPair.push_back(newPair);
    }

    // Create an interpolation object
    Interpolate a = Interpolate(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.5123,9.5123,25);
    std::chrono::nanoseconds interp_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        x.clear();
        x.push_back(xRangeI[i]);
        auto before = std::chrono::high_resolution_clock::now();
        ans = a.getInterpDer(x,0);
        auto after = std::chrono::high_resolution_clock::now();
        auto dt = after - before;
        interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
        ++count;

        std::cout << "I1: " << a.getInterpDer(x,0) << std::endl;
        std::cout << "I2: " << a.getInterpDerFunc(x,0) << std::endl;
        std::cout << "R:  " << 2*xRangeI[i] << std::endl;
    }
    interp_time /= count;
    std::cout << "interp nanos = " << interp_time.count() << std::endl;
}
