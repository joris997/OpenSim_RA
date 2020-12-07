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
    // Spring parameters
    double REST_LENGTH         { 1.0 };
    double STIFFNESS           { 500.0 };
    double DISSIPATION         { 0.1 };
    // Muscle parameters
    double OPT_FIBER_LENGTH    { 1.5 };         // was 0.55
    double TENDON_SLACK_LENGTH { 1.0 };         // was 0.50
    double MUSCLE_MAX_FORCE    { 6000.0 };      // was 4000.0
    // Sliding body parameters
    double BODY_MASS           { 10.0 };
    double BODY_MASS_FACTOR    { 1.0 };
    double BODY_SIZE           { 0.1 };
    double BODY_HEIGHT         { 1.08-0.05 };
    double BODY_OFFSET         { 0.5 };         // { 0.08+0.00001 };
    // Wrapping body parameters
    double CYLINDER_RADIUS     { 0.08 };
    double CYLINDER_HEIGHT     { 1.00 };
    int    CYLINDER_COUNT      { 5 };
//    double CYLINDER_HEIGHT     { 0.40 };
    SimTK::Vec3 CYLINDER_ROT   {0.0, 0.0, 0.0};
    std::string WRAP_BODY_TYPE { "cylinder" };
};

#endif //ANALYTICAL_WRAPPING_TESTCASE_H
