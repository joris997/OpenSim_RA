// Struct that contains variable parameters for the simple two-mass cylinder setup

#ifndef ANALYTICAL_WRAPPING_TESTCASE_H
#define ANALYTICAL_WRAPPING_TESTCASE_H

#include <OpenSim/OpenSim.h>

struct testCase {
    // General parameters
    bool   SHOW_VISUALIZER     { false };
    double REPORTING_INTERVAL  { 0.05 };
    double FINAL_TIME          { 5.0 };
    int    DISCRETIZATION      { 6 };
    bool   PATH_POINTS         { true };
    double OPT_FIBER_LENGTH    { 0.55 };         // was 0.55
    double TENDON_SLACK_LENGTH { 0.30 };         // was 0.50
    double MUSCLE_MAX_FORCE    { 4000.0 };
    // Sliding body parameters
    double BODY_SIZE           { 0.1 };
    double BODY_HEIGHT         { 1.08-0.05 };
    double BODY_OFFSET         { 0.4 };
//    double BODY_OFFSET         { 0.08+0.00001 };
    // Wrapping body parameters
    double CYLINDER_RADIUS     { 0.08 };
    double CYLINDER_HEIGHT     { 1.00 };
//    double CYLINDER_HEIGHT     { 0.40 };
    SimTK::Vec3 CYLINDER_ROT   {0.0, 0.0, 0.0};
};

#endif //ANALYTICAL_WRAPPING_TESTCASE_H
