#include <vector>
#include <iostream>
#include <chrono>
#include "interp.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using std::vector;
using std::pair;
using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::GeometryPath;

#include "testCase.h"
#include "testCaseDer.h"

int main(){
    testCase2D();
//    testCase2DDer();
//    testCase1DDer();
//    testCase2DOpenSim();
    return 0;
}





//void testCase1D(){
//    // 1D example
//    std::cout << "\n" << "1D test-case" << std::endl;

//    vector<double> xRange;
//    linspace(xRange,0,10,41);

//    vector<vector<double>> discretization1;
//    discretization1.push_back(xRange);

//    vector<pair<vector<int>,double>> evalsPair1;
//    for (int i=0; i<xRange.size(); i++){
//        vector<int> newIndex1;
//        newIndex1.push_back(i);
//        pair<vector<int>,double> newPair1;
//        newPair1.first = newIndex1;
//        newPair1.second = xRange[i]*xRange[i]*xRange[i];
//        evalsPair1.push_back(newPair1);
//    }
//    interp b = interp(discretization1,evalsPair1);

//    vector<double> x;
//    double valx = 3.3;
//    x.push_back(valx);
//    std::cout << "interpGrid: " << b.getInterp(x) <<
////                 " interpCubicHermite: " << b.interpCubicHermiteSpline(x,0) <<
//                 " should be: " << valx*valx*valx << std::endl;

//    std::cout << "dinterpGrid: " << b.getInterpDer(x,0) <<
////                 " dinterCubicHermite: " << b.interpCubicHermiteSpline(x,1) <<
//                 " should be: " << 3*valx*valx << std::endl;
//}
