#include <vector>
#include <iostream>
#include <chrono>
#include <OpenSim/OpenSim.h>
#include "SplineBasisFunction.h"
#include "MTUSplineInterface.h"
#include "MTUSplineDataRead.h"
#include "MTUSplineData.h"
#include "MTUSplineBase.h"
#include "MTUSpline.h"
#include "helperFunctions.h"

using std::vector;
using std::pair;
using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::GeometryPath;


int main(){
    // some quick test case to build an MTU spline
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

    // unlock coordinates
    bool c1_was_locked = c1.getLocked(st);
    c1.setLocked(st, false);
    auto unlock_c1 = defer_action([&] { c1.setLocked(st, c1_was_locked); });
    double c1_initial_value = c1.getValue(st);
    auto reset_c1_val = defer_action([&] { c1.setValue(st, c1_initial_value); });

    bool c2_was_locked = c2.getLocked(st);
    c2.setLocked(st, false);
    auto unlock_c2 = defer_action([&] { c2.setLocked(st, c2_was_locked); });
    double c2_initial_value = c2.getValue(st);
    auto reset_c2_val = defer_action([&] { c2.setValue(st, c2_initial_value); });

    // make discretization objects for interpolation class instance
    int num_steps = 12;
    Discretization dc1, dc2;
    vector<Discretization> discretization;
    dc1.begin    = c1.getRangeMin();
    dc1.end      = c1.getRangeMax();
    dc1.nPoints  = num_steps;
    dc1.gridsize = (dc1.end-dc1.begin)/dc1.nPoints;

    dc2.begin    = c2.getRangeMin();
    dc2.end      = c2.getRangeMax();
    dc2.nPoints  = num_steps;
    dc2.gridsize = (dc2.end-dc2.begin)/dc2.nPoints;

    discretization.push_back(dc1);
    discretization.push_back(dc2);

    // Create an interpolation object
    vector<int> discretizationNPoints;
    discretizationNPoints.push_back(num_steps);
    discretizationNPoints.push_back(num_steps);

    std::vector<double> a{dc1.begin,dc2.begin};
    std::vector<double> b{dc1.end,dc2.end};
    std::vector<int> n{dc1.nPoints,dc2.nPoints};

//    MTUSpline spline(a,b,n);
    MTUSplineInterface spline = MTUSplineInterface();
    spline.initialisation(muscle->getGeometryPath(),
                          std::move(affecting_coords),
                          st,
                          discretizationNPoints);

    // Create vectors of points to be evaluated
    vector<double> x;
    vector<double> xRange; linspace(xRange,dc1.begin,dc1.end,10);
    vector<double> yRange; linspace(yRange,dc2.begin,dc2.end,10);
    std::chrono::nanoseconds interp_time{0};
    std::chrono::nanoseconds interpS_time{0};
    std::chrono::nanoseconds real_time{0};
    int count = 0;
    vector<double> ansInterp;
    for (int i=0; i<xRange.size(); i++){
        for (int ii=0; ii<yRange.size(); ii++){
            x.clear();
            x.push_back(xRange[i]);
            x.push_back(yRange[ii]);

            auto before = std::chrono::high_resolution_clock::now();
            spline.setPosition(x);
            ansInterp = spline.getLMT();
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            ++count;
        }
    }

    double ansClassic;
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
            ansClassic = musc_path.getLength(st);
//            ans = musc_path.getLengtheningSpeed(s);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            real_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
        }
    }

    interp_time /= count;
    interpS_time /= count;
    real_time /= count;
    std::cout << "interp nanos       = " << interp_time.count() << std::endl;
    std::cout << "interpstruct nanos = " << interpS_time.count() << std::endl;
    std::cout << "real nanos         = " << real_time.count() << std::endl;
}
