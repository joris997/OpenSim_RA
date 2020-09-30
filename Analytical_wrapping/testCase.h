//
// Created by none on 20-09-20.
//

#ifndef ANALYTICAL_WRAPPING_TESTCASE_H
#define ANALYTICAL_WRAPPING_TESTCASE_H

#include <OpenSim/OpenSim.h>

struct testCase {
    // General parameters
    bool   SHOW_VISUALIZER   { true };
    double REPORTING_INTERVAL{ 0.2 };
    double FINAL_TIME        { 5.0 };
    // Sliding body parameters
    double BODY_SIZE         { 0.1 };
    double BODY_OFFSET       { 0.4 };
    // Wrapping body parameters
    double CYLINDER_RADIUS   { 0.08 };
    double CYLINDER_HEIGHT   { 0.5 };
    SimTK::Vec3 CYLINDER_ROT {0.0, 0.0, 0.0};
};

#endif //ANALYTICAL_WRAPPING_TESTCASE_H
