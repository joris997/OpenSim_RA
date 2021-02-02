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

void testCase2DOpenSim(){
    // 2D EXAMPLE
    std::cout << "\n" << "2D test-case" << std::endl;

    // get the model
    OpenSim::Model model{"Rajagopal_2015.osim"};
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
    int num_steps = 101;
    Discretization dc1, dc2;
    vector<Discretization> discretization;
    dc1.begin = c1.getRangeMin();
    dc1.end = c1.getRangeMax();
    dc1.nPoints = num_steps;
    dc1.gridsize = (dc1.end-dc1.begin) / dc1.nPoints;

    dc2.begin = c2.getRangeMin();
    dc2.end = c2.getRangeMax();
    dc2.nPoints = num_steps;
    dc2.gridsize = (dc2.end-dc2.begin) / dc2.nPoints;

    discretization.push_back(dc1);
    discretization.push_back(dc2);

    // Create an interpolation object
    interp a = interp(*muscle,
                      *affecting_coords[0],
                      *affecting_coords[1],
                      st,
                      discretization);

    vector<double> x;
    vector<double> xRange; linspace(xRange,dc1.begin,dc1.end,33);
    vector<double> yRange; linspace(yRange,dc2.begin,dc2.end,33);
    std::chrono::nanoseconds interp_time{0};
    std::chrono::nanoseconds interpS_time{0};
    std::chrono::nanoseconds real_time{0};
    int count = 0;
    double ans = 0;
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
            ans = a.getInterp(x);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            before = std::chrono::high_resolution_clock::now();
            ans = a.getInterpStruct(x);
            after = std::chrono::high_resolution_clock::now();
            dt = after - before;
            interpS_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            before = std::chrono::high_resolution_clock::now();
            ans = musc_path.getLength(st);
            after = std::chrono::high_resolution_clock::now();
            dt = after - before;
            real_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            std::cout << "I:  " << a.getInterp(x) << std::endl;
            std::cout << "Is: " << a.getInterpStruct(x) << std::endl;
            std::cout << "R:  " << musc_path.getLength(st) << std::endl;
            ++count;
        }
    }
    interp_time /= count;
    interpS_time /= count;
    real_time /= count;
    std::cout << "interp nanos       = " << interp_time.count() << std::endl;
    std::cout << "interpstruct nanos = " << interpS_time.count() << std::endl;
    std::cout << "real nanos         = " << real_time.count() << std::endl;
}


void testCase2D(){
    // 2D EXAMPLE
    std::cout << "\n" << "2D test-case" << std::endl;
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
    interp a = interp(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.5123,9.5123,25);
    std::chrono::nanoseconds interp_time{0};
    std::chrono::nanoseconds interpS_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        for (int ii=0; ii<xRangeI.size(); ii++){
            x.clear();
            x.push_back(xRangeI[i]); x.push_back(xRangeI[ii]);
//            auto before = std::chrono::high_resolution_clock::now();
//            ans = a.getInterp(x);
//            auto after = std::chrono::high_resolution_clock::now();
//            auto dt = after - before;
//            interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

//            before = std::chrono::high_resolution_clock::now();
//            ans = a.getInterpStruct(x);
//            after = std::chrono::high_resolution_clock::now();
//            dt = after - before;
//            interpS_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);

            ++count;

            std::cout << "I:  " << a.getInterp(x) << std::endl;
            std::cout << "Is: " << a.getInterpStruct(x) << std::endl;
            std::cout << "R:  " << xRangeI[i]*xRangeI[ii] << std::endl;
        }
    }
//    interp_time /= count;
//    interpS_time /= count;
//    std::cout << "interp nanos  = " << interp_time.count() << std::endl;
//    std::cout << "interpS nanos = " << interpS_time.count() << std::endl;
}


void testCase1D(){
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
    interp a = interp(discretization,evalsPair);

    vector<double> x;
    vector<double> xRangeI; linspace(xRangeI,0.5123,9.5123,25);
    std::chrono::nanoseconds interp_time{0};
    int count = 0;
    double ans = 0;
    for (int i=0; i<xRangeI.size(); i++){
        x.clear();
        x.push_back(xRangeI[i]);
        auto before = std::chrono::high_resolution_clock::now();
        ans = a.getInterp(x);
        auto after = std::chrono::high_resolution_clock::now();
        auto dt = after - before;
        interp_time += std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
        ++count;

        std::cout << "I:  " << a.getInterp(x) << std::endl;
//        std::cout << "Is: " << a.getInterpStruct(x) << std::endl;
        std::cout << "R:  " << xRangeI[i]*xRangeI[i] << std::endl;
    }
    interp_time /= count;
    std::cout << "interp nanos = " << interp_time.count() << std::endl;
}
