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

#include <string>
using std::to_string;
#include <vector>
using std::vector;
using std::pair;
#include <iostream>
using std::cout;
using std::endl;
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

#include "SplineDataNew.h"
#include "Spline.cpp"

//#define LOG

const int DIGIT_NUM = 8;
const int NUMBER_DIGIT_OUTPUT = 8;

inline double roundIt(double x, double n = DIGIT_NUM) {
    return floor( x * pow(10.0, n) + 0.5) / pow(10.0, n);
}

inline double radians (double d) {
    return d * M_PI / 180;
}

inline double degrees (double r) {
    return r * 180/ M_PI;
}

SplineData::SplineData(const vector<vector<double>>& disc,
                       const vector<pair<vector<double>,double>> evalPairs)
:a_(disc.size()), b_(disc.size()), n_(disc.size()){
    noDof_ = disc.size();
    noInputData_ = evalPairs.size();

    for (int i=0; i<noDof_; i++){
        muscleNames_.push_back(to_string(i));
        dofName_.push_back(to_string(i));

        a_[i] = disc[i][0];
        b_[i] = disc[i][disc[i].size()];
        n_[i] = disc[i].size();
    }

    // create vectors of all the angle combinations
    for (int i=0; i<noInputData_; i++){
        angles_.push_back(evalPairs[i].first);
    }

    // consider the muscles
    noMuscles_ = 1;        // this is actually incorrect, only 1 mus
    for (int i=0; i<noMuscles_; i++){
        y_.push_back(vector<double>(noInputData_));
    }
    for(int i=0; i<noInputData_; i++){
        for (int j=0; j<noMuscles_; j++){
            // this should evaluate the wrapping length
            y_[j][i] = evalPairs[i].second;
        }
    }

    // create the noMuscles_ splines
    for (int i = 0; i < noMuscles_; ++i) {
        Spline<N_DOF> newSpline(a_,b_,n_);
        splines_.push_back(newSpline);
    }

    // now compute coefficients for each muscle
    for (int i = 0; i < noMuscles_; ++i) {
        vector<double> currentMuscle(y_[i]);
        splines_[i].computeCoefficients(currentMuscle, currentMuscle.begin());
    }
}

SplineData::SplineData(const vector<pair<std::string,double[3]>>& dofInfo)
:dofName_(dofInfo.size()),  a_(dofInfo.size()), b_(dofInfo.size()), n_(dofInfo.size()) {
    noDof_ = dofInfo.size();

    noInputData_ = 1;
    for (int i=0; i<noDof_; i++){
        muscleNames_[i] = dofInfo[i].first;
        dofName_[i] = dofInfo[i].first;

        a_[i] = dofInfo[i].second[0];
        b_[i] = dofInfo[i].second[1];
        n_[i] = dofInfo[i].second[3];
        noInputData_ *= dofInfo[i].second[3];
    }

    // create vectors of all the angle combinations
    for (int i=0; i<noInputData_; i++){
        angles_.push_back(vector<double>(noDof_));
    }
    for (int i=0; i<noInputData_; i++){
        for (int j=0; j<noDof_; j++){
            // NEEDS A FIX
            angles_[i][j] = 0;
        }
    }

    // consider the muscles
    noMuscles_ = 1;        // this is actually incorrect, only 1 mus
    for (int i=0; i<noMuscles_; i++){
        y_.push_back(vector<double>(noInputData_));
    }
    // and create the 'wrapping length' evaluation. should later be incorporated
    // with the getWrappingLength() command of OpenSim
    for(int i=0; i<noInputData_; i++){
        for (int j=0; j<noMuscles_; j++){
            // this should evaluate the wrapping length
            y_[j][i] = (double)i*j;
        }
    }

    // create the noMuscles_ splines
    for (int i = 0; i < noMuscles_; ++i) {
        Spline<N_DOF> newSpline(a_,b_,n_);
        splines_.push_back(newSpline);
    }

    // now compute coefficients for each muscle
    for (int i = 0; i < noMuscles_; ++i) {
        vector<double> currentMuscle(y_[i]);
        splines_[i].computeCoefficients(currentMuscle, currentMuscle.begin());
    }
}


double SplineData::getEval(std::vector<double> x){
    return splines_[0].getValue(x);
}


vector<double> SplineData::getEvalDer(std::vector<double> x){
    vector<double> derivatives;
    for (int k=0; k<noDof_; ++k){
        derivatives.push_back(-splines_[0].getFirstDerivative(x,k));
    }
    return derivatives;
}
