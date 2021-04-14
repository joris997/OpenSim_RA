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
#include <stdlib.h>

#include "SplineData.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;


int main(int argc, const char* argv[]) 
{
    // get the model
    OpenSim::Model model{"/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/RajagopalModel/Rajagopal2015.osim"};
    SimTK::State& st = model.initSystem();
    model.equilibrateMuscles(st);
    model.realizeVelocity(st);

    // get the muscle
    const Muscle* muscle;
    for (Muscle const& m : model.getComponentList<Muscle>()){
        if (m.getName() == "soleus_r"){
            muscle = &m;
        }
    }
    // get the coordinates
    vector<Coordinate const*> coords;
    int max_coord_namelen = 0;
    for (Coordinate const& c : model.getComponentList<Coordinate>()) {
        if (c.getMotionType() != Coordinate::MotionType::Coupled) {
            max_coord_namelen = std::max(max_coord_namelen, static_cast<int>(c.getName().size()));
            coords.push_back(&c);
        }
    }
    vector<Coordinate const*> affecting_coords;
    Nonzero_conditions cond;
    for (Coordinate const* c : coords){
        if(coord_affects_muscle(*muscle,*c,st,cond)){
            affecting_coords.push_back(c);
        }
    }
    GeometryPath const& musc_path = muscle->getGeometryPath();
    Coordinate const& c1 = *affecting_coords[0];
    Coordinate const& c2 = *affecting_coords[1];

    // Create an interpolation object
    vector<int> nPoints;
    nPoints.push_back(8);
    nPoints.push_back(8);

    SplineData spline = SplineData(muscle->getGeometryPath(),
                                   std::move(affecting_coords),
                                   st,
                                   nPoints);

    // Create vectors of points to be evaluated
    vector<double> x;
    vector<double> xRange; linspace(xRange,-0.1,0.1,4);
    vector<double> yRange; linspace(yRange,-0.1,0.1,4);
    std::chrono::nanoseconds interp_time{0};
    std::chrono::nanoseconds real_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            x.clear();
            x.push_back(xRange[i]);
            x.push_back(yRange[ii]);

            auto before = std::chrono::high_resolution_clock::now();
            ans = spline.evalLmt(x);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            ++count;
        }
    }
    for (int i=0; i<xRange.size(); i++){
        double c1v = xRange[i];
        c1.setValue(st, c1v);

        for (int ii=0; ii<yRange.size(); ii++){
            double c2v = yRange[i];
            c2.setValue(st, c2v);

            x.clear();
            x.push_back(xRange[i]);
            x.push_back(yRange[ii]);

            auto before = std::chrono::high_resolution_clock::now();
            ans = musc_path.getLength(st);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            real_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            std::cout << "interp state:   " << spline.evalLmt(x) << std::endl;
            std::cout << "real:           " << musc_path.getLength(st) << std::endl;
        }
    }

    interp_time /= count;
    real_time /= count;
    std::cout << "interp nanos       = " << interp_time.count() << std::endl;
    std::cout << "real nanos         = " << real_time.count() << std::endl;

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
