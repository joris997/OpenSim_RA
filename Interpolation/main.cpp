#include <vector>
#include <iostream>
#include <chrono>
#include "interp.h"

using std::vector;
using std::pair;
//#include <OpenSim/OpenSim.h>


template<typename T>
void linspace(vector<double> &linspaced, T start_in, T end_in, int num_in){
// from: https://stackoverflow.com/questions/27028226/python-linspace-in-c
  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0){
      return;
  }
  if (num == 1){
      linspaced.push_back(start);
      return;
  }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i){
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
}


void testCase2D(){
    // 2D EXAMPLE
    std::cout << "\n" << "2D test-case" << std::endl;
    // Create the interpolation field
    vector<double> xRange;
    vector<double> yRange;
    linspace(xRange,0,10,121);
    linspace(yRange,0,10,155);

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
    interp a = interp(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.5123,9.5123,25);
    std::chrono::microseconds interp_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        for (int ii=0; ii<xRangeI.size(); ii++){
            x.clear();
            x.push_back(xRangeI[i]); x.push_back(xRangeI[ii]);
            auto before = std::chrono::high_resolution_clock::now();
            ans = a.getInterp(x);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            interp_time += std::chrono::duration_cast<std::chrono::microseconds>(dt);
            ++count;

            std::cout << "I: " << a.getInterp(x) << std::endl;
            std::cout << "R: " << xRangeI[i]*xRangeI[ii] << std::endl;
        }
    }
    interp_time /= count;
    std::cout << "interp micros = " << interp_time.count() << std::endl;

//    double valx = 5.3; double valy = 5.6;
//    x.push_back(valx); x.push_back(valy);
//    std::cout << "interpGrid: " << a.getInterp(x) <<
////                 " interpCubicHermite: " << a.interpCubicHermiteSpline(x,0) <<
//                 " should be: " << valx*valy << std::endl;

//    std::cout << "dinterGrid: " << a.getInterpDer(x,0) <<
////                 " dinterCubicHermite: " << a.interpCubicHermiteSpline(x,1) <<
//                 " should be: " << valy << std::endl;
}


void testCase1D(){
    // 1D example
    std::cout << "\n" << "1D test-case" << std::endl;

    vector<double> xRange;
    linspace(xRange,0,10,41);

    vector<vector<double>> discretization1;
    discretization1.push_back(xRange);

    vector<pair<vector<int>,double>> evalsPair1;
    for (int i=0; i<xRange.size(); i++){
        vector<int> newIndex1;
        newIndex1.push_back(i);
        pair<vector<int>,double> newPair1;
        newPair1.first = newIndex1;
        newPair1.second = xRange[i]*xRange[i]*xRange[i];
        evalsPair1.push_back(newPair1);
    }
    interp b = interp(discretization1,evalsPair1);

    vector<double> x;
    double valx = 3.3;
    x.push_back(valx);
    std::cout << "interpGrid: " << b.getInterp(x) <<
//                 " interpCubicHermite: " << b.interpCubicHermiteSpline(x,0) <<
                 " should be: " << valx*valx*valx << std::endl;

    std::cout << "dinterpGrid: " << b.getInterpDer(x,0) <<
//                 " dinterCubicHermite: " << b.interpCubicHermiteSpline(x,1) <<
                 " should be: " << 3*valx*valx << std::endl;
}

void testCaseOpenSim(){

}


int main(){
    bool test = true;
    if (test){
//        testCase1D();
        testCase2D();
    } else {
        testCaseOpenSim();
    }
    return 0;
}
