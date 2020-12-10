// Copyright (c) 2011, Massimo Sartori and Monica Reggiani
// All rights reserved.

// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, 
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <vector>
using std::vector;
using std::pair;
#include <stdlib.h>

#include "SplineDataNew.h"

void linspace(std::vector<double> &vector, double begin, double end, int nPoints){
    // TODO: also incorporate begin so it does not have to begin with zero
    for (int i=0; i<nPoints; i++){
        vector.push_back((double)end/(nPoints-1)*i);
    }
}

int main(int argc, const char* argv[]) 
{
    // 2D EXAMPLE
    std::cout << "\n" << "2D test-case" << std::endl;
    // Create the interpolation field
    vector<double> xRange;
    vector<double> yRange;
    linspace(xRange,0,10,11);
    linspace(yRange,0,10,11);

    vector<vector<double>> discretization;
    discretization.push_back(xRange);
    discretization.push_back(yRange);

    // Create evaluations of wrapping length functions
    vector<pair<vector<double>,double>> evalsPair;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            vector<double> newIndex;
            newIndex.push_back(xRange[i]); newIndex.push_back(yRange[ii]);
            pair<vector<double>,double> newPair;
            newPair.first = newIndex;
            newPair.second = xRange[i]*yRange[ii];
            evalsPair.push_back(newPair);
        }
    }

    // Create SplineData object
    SplineData splineData(discretization, evalsPair);
    // vector at which we would like to evaluate
    vector<double> x1;
    x1.push_back(1.0); x1.push_back(2.0); x1.push_back(3.0);
    cout << "eval x1: " << splineData.getEval(x1) << endl;
    for (int i=0; i<x1.size(); i++){
        cout << "deval x1: " << splineData.getEvalDer(x1)[i] << endl;
    }

    vector<double> x2;
    x2.push_back(1.2); x2.push_back(2.2); x2.push_back(3.2);
    cout << "eval x2: " << splineData.getEval(x2) << endl;
    for (int i=0; i<x2.size(); i++){
        cout << "deval x2: " << splineData.getEvalDer(x2)[i] << endl;
    }



//    // NEW METHOD OF CALLING
//    vector<pair<string,double[3]>> dofInfo;
//    vector<string> dofNames {"arm_flex_r","arm_add_r","arm_rot_r"};
//    for (int i=0; i<dofNames.size(); i++){
//        pair<string,double[3]> newPair;
//        newPair.first = dofNames[i];
//        double newPairSecond[3] = {0.0,10.0,11.0};
//        newPair.second = newPairSecond;
//        dofInfo.push_back(newPair);
//    }
//    SplineData splineData(dofInfo);

//    // vector at which we would like to evaluate
//    vector<double> x1;
//    x1.push_back(1.0); x1.push_back(2.0); x1.push_back(3.0);
//    // evaluate lmt
//    cout << "eval x1: " << splineData.getEval(x1) << endl;
//    // evaluate moment arm
//    cout << "deval x1: " << splineData.getEvalDer(x1) << endl;

//    vector<double> x2;
//    x2.push_back(1.2); x2.push_back(2.2); x2.push_back(3.2);
//    // now evaluate Lmt
//    splineData.getEval(x2);
//    // and evaluate Ma
//    splineData.getEvalDer(x2);


    return 0;

//    // Build a set of spline (one for each muscle)
//    // reading the input data from the "lmt.in" file

//    string dataDirectory = argv[1];
//    string inputDataFilename = dataDirectory + "InputData/lmt.in";

//    SplineData splineData(inputDataFilename);

//    // Now use the spline to evaluate lmt & ma on the nodes
//    // used as input to build the spline
//    string evalDataDir;
//    evalDataDir =  dataDirectory + "NodesData/";

//    splineData.setEvalDataDir(evalDataDir);
//    // get the angles
//    splineData.readEvalAngles();

//    cout << "Eval data for Lmt on Nodes\n";
//    // now evaluate Lmt
//    splineData.evalLmt();

//    cout << "Eval data for Ma on Nodes\n";
//    // and evaluate Ma
//    splineData.evalMa();


//    // Now use the spline to evaluate lmt & ma on the nodes
//    // between the ones used as input to build the spline
//    evalDataDir =  dataDirectory + "BetweenNodesData/";

//    splineData.setEvalDataDir(evalDataDir);
//    // get the angles
//    splineData.readEvalAngles();

//    cout << "Eval data for Lmt between Nodes\n";
//    // now evaluate Lmt
//    splineData.evalLmt();

//    cout << "Eval data for Ma between Nodes\n";
//    // and evaluate Ma
//    splineData.evalMa();
//    exit(EXIT_SUCCESS);
}