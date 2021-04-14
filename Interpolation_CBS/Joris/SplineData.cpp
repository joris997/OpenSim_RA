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

#include <sstream>
using std::stringstream;
#include <iostream>
using std::cout;
using std::endl;
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <iomanip>

#include "SplineData.h"
#include "Spline.cpp"

#define LOG

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

//////////
/// \brief SplineData constructor
/// \param gp: geometry path
/// \param cBegin: first coordinate
/// \param cEnd: last coordinate
/// \param st: state of the system
/// \param nPoints: vector of number of discretization points
SplineData::SplineData(OpenSim::GeometryPath const& gp,
                       OpenSim::Coordinate const** cBegin,
                       OpenSim::Coordinate const** cEnd,
                       SimTK::State& st,
                       std::vector<int>& nPoints)
    :dofName_(N_DOF),  a_(N_DOF), b_(N_DOF), n_(N_DOF), g_(N_DOF) {
    getInputData(gp,cBegin,cEnd,st,nPoints);

    // create the noMuscles_ splines
    Spline<N_DOF> newSpline(a_,b_,n_);
    splines_.push_back(newSpline);

    // now compute coefficients for each muscle
    vector<double> currentMuscle(y_[0]);
    splines_[0].computeCoefficients(currentMuscle, currentMuscle.begin());
}

//////////
/// \brief SplineData wrapper constructer
/// \param pbp: point-based path input
/// \param coords: vector of coordinates
/// \param st: state of the system
/// \param nPoints: vector of number of discretization points
SplineData::SplineData(OpenSim::GeometryPath const& pbp,
                       std::vector<OpenSim::Coordinate const*> coords,
                       SimTK::State& st,
                       std::vector<int>& nPoints)
    : SplineData(pbp,
                 &coords[0],
                 &coords[coords.size()-1],
                 st,
                 nPoints){
}

SplineData::SplineData(const string& inputDataFilename)
:inputDataFile_(inputDataFilename.c_str()), dofName_(N_DOF),  a_(N_DOF), b_(N_DOF), n_(N_DOF), g_(N_DOF) {
    if (!inputDataFile_.is_open()) {
        cout << "ERROR: " << inputDataFilename << " could not be open\n";
        exit(EXIT_FAILURE);
    }

#ifdef LOG
  cout << "Reading input data from: " 
       << inputDataFilename << endl;
#endif

    readInputData();
    inputDataFile_.close();

#ifdef LOG
  cout << "Read the following interpolation data:\n";
  displayInputData(); 
#endif

    // create the noMuscles_ splines
    for (int i = 0; i < noMuscles_; ++i) {
        Spline<N_DOF> newSpline(a_,b_,n_);
        splines_.push_back(newSpline);
    }

#ifdef LOG
  cout << "Created " << splines_.size() << " splines.\n";
#endif     

    // now compute coefficients for each muscle
    for (int i = 0; i < noMuscles_; ++i) {
        vector<double> currentMuscle(y_[i]);
        splines_[i].computeCoefficients(currentMuscle, currentMuscle.begin());
    }

}


void SplineData::getInputData(OpenSim::GeometryPath const& gp,
                              OpenSim::Coordinate const** cBegin,
                              OpenSim::Coordinate const** cEnd,
                              SimTK::State& st,
                              std::vector<int>& nPoints){
    // get the size of the 'vector'. Inclusive bottom, exclusive top
    std::ptrdiff_t n = (cEnd-cBegin)+1;
    assert(n > 0);
    assert(dimension == (int)n);

    // put all coordinate pointers in a vector to later unpack an incoming
    // state to a vector of coordinate values
    for (int i=0; i<N_DOF; i++){
        coords.push_back(cBegin[i]);
    }

    // unlock coordinates
    for (int i=0; i<N_DOF; i++){
        const OpenSim::Coordinate& c = *cBegin[i];
        bool c_was_locked = c.getLocked(st);
        c.setLocked(st, false);
        auto unlock_c = defer_action([&] { c.setLocked(st, c_was_locked); });
        double c_initial_value = c.getValue(st);
        auto reset_c_val = defer_action([&] { c.setValue(st, c_initial_value); });
    }

    // make discretization objects for interpolation class instance
    Discretization dc;
    for (int i=0; i<N_DOF; i++){
        const OpenSim::Coordinate& c = *cBegin[i];
        a_[i] = c.getRangeMin();
        b_[i] = c.getRangeMax();
        n_[i] = nPoints[i];
        g_[i] = (b_[i]-a_[i]) / n_[i];
    }

    // slightly extend the bound for accurate interpolation on the edges
    for (int dim=0; dim<N_DOF; dim++){
        a_[dim] -= 1*g_[dim];
        b_[dim] += 2*g_[dim];
        n_[dim] += 3;
    }

    int numOfLoops = 1;
    for (int dim=0; dim<N_DOF; dim++){
        numOfLoops *= n_[dim];
    }
    std::cout << "numOfLoops: " << numOfLoops << std::endl;
    std::vector<int> cnt(N_DOF);
    std::vector<double> coordValues(N_DOF);
    std::vector<double> y_vector;
    for (int i=0; i<numOfLoops; i++){
        for (int ii=0; ii<N_DOF; ii++){
            coordValues[ii] = a_[ii] + (cnt[ii]*g_[ii]);
            const OpenSim::Coordinate& c = *cBegin[ii];
            c.setValue(st,coordValues[ii]);
        }
        y_vector.push_back(gp.getLength(st));

        // update cnt values
        for (int x=N_DOF-1; x>=0; x--){
            if (cnt[x] != n_[x]-1){
                cnt[x] += 1;
                break;
            } else{
                for (int y=x; y<N_DOF; y++){
                    cnt[y] = 0;
                }
            }
        }
    }
    y_.push_back(y_vector);
}


void SplineData::readInputData() {
    // --- Read DOFs
    for (int i=0; i < N_DOF; ++i) {
        inputDataFile_ >> dofName_[i];
        inputDataFile_ >> a_[i]; a_[i] = radians(a_[i]);
        inputDataFile_ >> b_[i]; b_[i] = radians(b_[i]);
        inputDataFile_ >> n_[i];
    }

    string line;
    getline(inputDataFile_, line, '\n'); getline(inputDataFile_, line, '\n');
    stringstream myStream(line);
    string nextMuscleName;
    // --- Read Interpolation Data
    // 1. first their names
    do {
        myStream >> nextMuscleName;
        muscleNames_.push_back(nextMuscleName);
    } while (!myStream.eof());

    noMuscles_ = muscleNames_.size();

    // 2. then their values for all the possible combination of DOFs values
    // 2a. create the matrix to store them
    noInputData_ = 1;
    for (int i = 0; i < N_DOF; ++i){
        noInputData_ *= ( n_[i]+1 );
    }
    for (int i = 0; i < noMuscles_; ++i){
        y_.push_back(vector<double>(noInputData_));
    }

    // 2b. read the data
    for (int j=0; j < noInputData_; ++j){
        for (int i=0; i < noMuscles_; ++i) {
            inputDataFile_ >> y_[i][j];
        }
    }
}


void SplineData::displayInputData() {
    cout << "-- DOFs  \n";
    cout << "DofName\t a \t b \t n \t h \n";
    for (int i=N_DOF-1; i >=0; --i) {
        cout << dofName_[i] << "\t";
        cout << a_[i] << "\t" ;
        cout << b_[i] << "\t" ;
        cout << n_[i] << "\t" ;
        cout << (b_[i]-a_[i])/n_[i] << endl;
    }

    cout << "-- Data \n";
    for (int i = N_DOF-1; i >=0; --i){
        cout << dofName_[i] << "\t";
    }

    for (int i=0; i < noMuscles_; ++i) {
        cout << muscleNames_[i] << "\t";
    }
    cout << endl;
 
    vector<int> index(N_DOF);
    double tot, mul;
    for (int soFar = 0; soFar < noInputData_; ++soFar) {
        tot = 0; mul = noInputData_;
        for (int i = N_DOF-1; i >0; --i) {
            mul = mul / ( n_[i] + 1 );
            index[i] = ( soFar - tot )/ mul;
            tot += index[i] * mul;
        }
        index[0] = soFar % ( n_[0] + 1 );
        for (int i = N_DOF-1; i >= 0; --i) {
            cout << a_[i] + index[i] * (b_[i]-a_[i])/n_[i] << "\t";
        }
        for (int j = 0; j < noMuscles_; ++j){
            cout << y_[j][soFar] << "\t";
        }
        cout << endl;
    }
}


void SplineData::readEvalAngles() {
    string anglesFilename = evalDataDir_ + "/angles.in";
    ifstream anglesFile(anglesFilename.c_str());

    if (!anglesFile.is_open()) {
        cout << "ERROR: " << anglesFilename << " could not be open\n";
        exit(EXIT_FAILURE);
    }

    anglesFile >> noEvalData_;
    for (int i = 0; i < noEvalData_; ++i){
        angles_.push_back(vector<double>(N_DOF));
    }

    for (int i=0; i < noEvalData_; ++i){
        for (int j=N_DOF-1; j>=0; --j) {
            anglesFile >> angles_[i][j]; angles_[i][j] = radians(angles_[i][j]);
        }
    }
    anglesFile.close();
}



void SplineData::openOutputFile(ofstream& outputDataFile){
    if (!outputDataFile.is_open()) {
        cout << "ERROR: outputDataFile could not be open\n";
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < noMuscles_; ++i){
        outputDataFile << muscleNames_[i] << "\t";
    }
    outputDataFile << endl;

    for (int i = 0; i < noMuscles_; ++i){
        outputDataFile << "eval\t";
    }
    outputDataFile << endl;
}  

void SplineData::openEvalFile(ifstream& evalDataFile) {
    if (!evalDataFile.is_open()) {
        cout << "ERROR: evalData File could not be open\n";
        exit(EXIT_FAILURE);
    }

    // check we have the same amount of data of angles
    int numRows;
    evalDataFile >> numRows;

    if (numRows != noEvalData_) {
        cout << "ERROR: we have " << noEvalData_ << " angles, but " << numRows
        << " lines of data in input from the evalDataFile"  << endl;
        exit(EXIT_FAILURE);
    }

    // check we have the same muscles
    string line;
    vector<string> muscleNames;
    getline(evalDataFile, line, '\n'); getline(evalDataFile, line, '\n');
    stringstream myStream(line);
    string nextMuscle;

    do {
        myStream >> nextMuscle;
        muscleNames.push_back(nextMuscle);
    } while (!myStream.eof());

    if (muscleNames.size() != static_cast<unsigned int>(noMuscles_)) {
        cout << "ERROR: we have " << noMuscles_ << " interpolated muscles, but " << muscleNames.size()
        << " muscles in the evalDataFile" << endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0 ; i < noMuscles_; ++i) {
        if (muscleNames[i] != muscleNames_[i]) {
            cout << "ERROR: the " << i << "-th muscle is "
            << muscleNames_[i] << " in the interpolated Data while"
            << muscleNames[i] << " in file evalData " << endl;
            exit(EXIT_FAILURE);
        }
    }
}

double SplineData::evalLmt(vector<double> x){
    return splines_[0].getValue(x);
}

void SplineData::evalLmt() {
    // First open the inputDataFile
    string evalDataFilename = evalDataDir_ + "lmt.in";
    ifstream evalDataFile(evalDataFilename.c_str());

    openEvalFile(evalDataFile);

    // Then open the outputDataFile

    string outputDataFilename = evalDataDir_ + "lmt.out";
    ofstream outputDataFile(outputDataFilename.c_str());
    openOutputFile(outputDataFile);

    // now readData

    double nextValue;
    for (int j = 0; j < noEvalData_; ++j) {
        for (int i = 0; i < noMuscles_; ++i) {
            // we round the results at the number of digits of the input file
            evalDataFile >> nextValue;
            // outputDataFile <<  std::setprecision(NUMBER_DIGIT_OUTPUT) << std::fixed << nextValue << "\t";
            outputDataFile <<  std::setprecision(NUMBER_DIGIT_OUTPUT) << std::fixed << roundIt(splines_[i].getValue(angles_[j]), DIGIT_NUM+2) << "\t";
        }
        outputDataFile << endl;
    }
    outputDataFile.close();
}


double SplineData::evalMa(vector<double> x, int coord){
    return splines_[0].getFirstDerivative(x,coord);
}

void SplineData::evalMa() {
  // for all the degree of freedom
  for (int k = 0; k < N_DOF; ++k) {
      
    // First open the inputDataFile
    string evalDataFilename = evalDataDir_ + "ma" + dofName_[k] + ".in";  
    ifstream evalDataFile(evalDataFilename.c_str());
  
    openEvalFile(evalDataFile);
    
    // Then open the outputDataFile
    string outputDataFilename = evalDataDir_ +  "ma" + dofName_[k] + ".out";  
    ofstream outputDataFile(outputDataFilename.c_str());
    openOutputFile(outputDataFile);

    double nextValue;
    for (int j = 0; j < noEvalData_; ++j) {
      for (int i = 0; i < noMuscles_; ++i) {  
        evalDataFile >> nextValue;
      //  outputDataFile << std::setprecision(NUMBER_DIGIT_OUTPUT) << std::fixed << nextValue << "\t";
        outputDataFile << std::setprecision(NUMBER_DIGIT_OUTPUT) << std::fixed << -roundIt(splines_[i].getFirstDerivative(angles_[j],k), DIGIT_NUM+2) << "\t";
      } 
      outputDataFile << endl;
    }
    outputDataFile.close();
  }
}

