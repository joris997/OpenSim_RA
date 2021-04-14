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


#ifndef SplineData_h
#define SplineData_h

#include <OpenSim/OpenSim.h>
#include "Spline.h"

#include <vector>
using std::vector;
#include <string>
using std::string;

#include <fstream>
using std::ifstream;
using std::ofstream;

const int N_DOF = 2;


struct Discretization{
    double begin;
    double end;
    int nPoints;
    double gridsize;
};
struct Nonzero_conditions final {
    double input_val;
    double nonzero_ma;
};

/////////////////////
// helper functions
/////////////////////
template<typename T>
void linspace(std::vector<double> &linspaced, T start_in, T end_in, int num_in){
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

// defer an action until the destruction of this wrapper
template<typename Callback>
struct Defer final {
    Callback cb;
    Defer(Callback _cb) : cb{std::move(_cb)} {
    }
    Defer(Defer const&) = delete;
    Defer(Defer&&) noexcept = default;
    Defer& operator=(Defer const&) = delete;
    Defer& operator=(Defer&&) = delete;
    ~Defer() noexcept {
        cb();
    }
};

template<typename Callback>
Defer<Callback> defer_action(Callback cb) {
    return Defer<Callback>{std::move(cb)};
}

static bool coord_affects_muscle(
    OpenSim::Muscle const& m,
    OpenSim::Coordinate const& c,
    SimTK::State& state,
    Nonzero_conditions& out) {

    bool prev_locked = c.getLocked(state);
    auto reset_locked = defer_action([&] { c.setLocked(state, prev_locked); });
    double prev_val = c.getValue(state);
    auto reset_val = defer_action([&] { c.setValue(state, prev_val); });

    c.setLocked(state, false);

    static constexpr int num_steps = 3;
    double start = c.getRangeMin();
    double end = c.getRangeMax();
    double step = (end - start) / num_steps;

    for (double v = start; v <= end; v += step) {
        c.setValue(state, v);
        double ma = m.getGeometryPath().computeMomentArm(state, c);
        if (std::abs(ma) > 0.001) {
            out.input_val = v;
            out.nonzero_ma = ma;
            return true;
        }
    }
    return false;
}

/////////////////////////////////
/// \brief The SplineData class
/////////////////////////////////
class SplineData { 
public:
    SplineData(OpenSim::GeometryPath const& pbp,
               std::vector<OpenSim::Coordinate const*> coords,
               SimTK::State& st,
               std::vector<int>& nPoints);
    SplineData(OpenSim::GeometryPath const& gp,
               OpenSim::Coordinate const** cBegin,
               OpenSim::Coordinate const** cEnd,
               SimTK::State& st,
               std::vector<int>& discretizationNPoints);
    SplineData(const string& inputDataFilename);
    void setEvalDataDir(const string& evalDataDir) {evalDataDir_ = evalDataDir;}
    void readEvalAngles();
    void evalLmt();
    double evalLmt(vector<double> x);
    void evalMa();
    double evalMa(vector<double>x, int coord);
private:
    void getInputData(OpenSim::GeometryPath const& gp,
                      OpenSim::Coordinate const** cBegin,
                      OpenSim::Coordinate const** cEnd,
                      SimTK::State& st,
                      std::vector<int>& nPoints);
    void readInputData();
    void displayInputData();
    void openEvalFile(ifstream& evalDataFile);
    void openOutputFile(ofstream& outputDataFile);
    ifstream inputDataFile_;

    // Interpolation Data
    vector<string> dofName_;
    vector<double> a_;
    vector<double> b_;
    vector<int> n_;
    vector<double> g_;
    vector<string> muscleNames_;
    vector<const OpenSim::Coordinate *> coords;
    int noMuscles_;
    int noInputData_;
    vector< vector< double> > y_;

    // Splines
    vector< Spline<N_DOF> > splines_;

    // EvalData
    string evalDataDir_;
    int noEvalData_;
    vector < vector <double> > angles_;
};


#endif
