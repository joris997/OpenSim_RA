#include <vector>
#include <iostream>
#include "interp.h"
//#include <OpenSim/OpenSim.h>

void linspace(std::vector<double> &vector, double begin, double end, int nPoints){
    // TODO: also incorporate begin so it does not have to begin with zero
    for (int i=0; i<nPoints; i++){
        vector.push_back((double)end/(nPoints-1)*i);
    }
}


void testCase2D(){
    // 2D EXAMPLE
    std::cout << "\n" << "2D test-case" << std::endl;
    // Create the interpolation field
    std::vector<double> xRange;
    std::vector<double> yRange;
    linspace(xRange,0,10,11);
    linspace(yRange,0,10,11);

    std::vector<std::vector<double>> discretization;
    discretization.push_back(xRange);
    discretization.push_back(yRange);

    // Create evaluations of wrapping length functions
    std::vector<std::pair<std::vector<int>,double>> evalsPair;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            std::vector<int> newIndex;
            newIndex.push_back(i); newIndex.push_back(ii);
            std::pair<std::vector<int>,double> newPair;
            newPair.first = newIndex;
            newPair.second = xRange[i]*yRange[ii];
            evalsPair.push_back(newPair);
        }
    }

    // Create an interpolation object
    interp a = interp(discretization,evalsPair);

    std::vector<double> x;
    double valx = 5.3; double valy = 5.6;
    x.push_back(valx); x.push_back(valy);
    std::cout << "interpGrid: " << a.interpGridSpline(x) <<
                 " interpCubicHermite: " << a.interpCubicHermiteSpline(x,0) <<
                 " should be: " << valx*valy << std::endl;

    std::cout << "derivative: " << a.interpCubicHermiteSpline(x,1) <<
                 " should be: " << valx + valy << std::endl;
}


void testCase1D(){
    // 1D example
    std::cout << "\n" << "1D test-case" << std::endl;

    std::vector<double> xRange;
    linspace(xRange,0,10,11);

    std::vector<std::vector<double>> discretization1;
    discretization1.push_back(xRange);

    std::vector<std::pair<std::vector<int>,double>> evalsPair1;
    for (int i=0; i<xRange.size(); i++){
        std::vector<int> newIndex1;
        newIndex1.push_back(i);
        std::pair<std::vector<int>,double> newPair1;
        newPair1.first = newIndex1;
        newPair1.second = xRange[i]*xRange[i];
        evalsPair1.push_back(newPair1);
    }
    interp b = interp(discretization1,evalsPair1);

    std::vector<double> x1;
    double valx1 = 3.1;
    x1.push_back(valx1);
    std::cout << "interpGrid: " << b.interpGridSpline(x1) <<
                 " interpCubicHermite: " << b.interpCubicHermiteSpline(x1,0) <<
                 " should be: " << valx1*valx1 << std::endl;

    std::cout << "derivative: " << b.interpCubicHermiteSpline(x1,1) <<
                 " should be: " << 2*valx1 << std::endl;
}

void testCaseOpenSim(){

}


int main(){
    bool test = true;
    if (test){
        testCase1D();
        testCase2D();
    } else {
        testCaseOpenSim();
    }
    return 0;
}
