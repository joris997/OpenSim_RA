/* -------------------------------------------------------------------------- *
 *                     test_suite.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Test_suite that checks analytical solutions of muscle lengths over         *
 * wrapping surfaces with the numerical solutions of OpenSim. Especially      *
 * important for unconventional wrapping (over a rotated cylinder for example *
 *                                                                            *
 * Author(s): Joris Verhagen                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include <OpenSim/OpenSim.h>
#include "testCase.h"
#include <ctime>
#include <algorithm>
#include <vector>
#include <fstream>
#include <regex>

static const std::string sliderLPath{"/jointset/sliderLeft/yCoordSliderLeft"};
static const std::string sliderRPath{"/jointset/sliderRight/yCoordSliderRight"};
static const char visualizer_arg[] = "--visualizer";
static const char cylinder_radius_arg[] = "--cylinder_radius=";
static const char stiffness_arg[] = "--stiffness=";
static const char dissipation_arg[] = "--dissipation=";
static const char body_mass_arg[] = "--body_mass=";
static const char body_mass_factor_arg[] = "--body_mass_factor=";
static const char cylinder_rotation_arg[] = "--cylinder_rotation=";
static const char wrapping_body_type_arg[] = "--type=";
static const char final_time_arg[] = "--final_time=";
static const char amplitude_arg[] = "--amplitude=";

using OpenSim::Exception;
using OpenSim::Model;
using OpenSim::ConsoleReporter;
using SimTK::Vec3;

/*
Model buildWrappingModel(const testCase& tc);
int main(int argc, char** argv) {
    using namespace OpenSim;

    testCase tc;
    auto model = buildWrappingModel(tc);
    model.finalizeConnections();

    SimTK::State& x0 = model.initSystem();

    simulate(model, x0, tc.FINAL_TIME, true);

    return 0;
}
*/

Model buildWrappingModel(const testCase& tc);
int test(const testCase& tc);

// if this returns int and you check the value of the int you can analyze errors
// happening in this function. While it is void, you cannot.
void commandLineArguments(testCase& tc, int argc, char** argv){
    char** args = argv + 1;
    for (int nargs = argc-1; nargs>0; --nargs, ++args){
        char const* arg = *args;
        // visualizer
        if (strcmp(arg,visualizer_arg) == 0){
            tc.SHOW_VISUALIZER = true;
        }
        // wrap object rotation
        else if (strncmp(arg,cylinder_rotation_arg,sizeof(cylinder_rotation_arg)-1)==0){
            char const* val = arg + (sizeof(cylinder_rotation_arg)-1);
            std::cmatch m;
            std::regex pattern{R"(\[([^,]+),([^,]+),([^\]]+)\])"};
            if (std::regex_match(val,m,pattern)){
                char* end;
                tc.CYLINDER_ROT = Vec3{strtod(m[1].str().c_str(), &end),
                                       strtod(m[2].str().c_str(), &end),
                                       strtod(m[3].str().c_str(), &end)};
            }
        }
        // wrap object type
        else if (strncmp(arg,wrapping_body_type_arg,sizeof(wrapping_body_type_arg)-1)==0){
            char const* val = arg + (sizeof(wrapping_body_type_arg)-1);
            tc.WRAP_BODY_TYPE = val;
        }
        // cylinder radius
        else if (strncmp(arg,cylinder_radius_arg,sizeof(cylinder_radius_arg)-1)==0){
            char const* val = arg + (sizeof(cylinder_radius_arg)-1);
            char* end;
            tc.CYLINDER_RADIUS = strtod(val, &end);
        }
        // spring stiffness
        else if (strncmp(arg,stiffness_arg,sizeof(stiffness_arg)-1)==0){
            char const* val = arg + (sizeof(stiffness_arg)-1);
            char* end;
            tc.STIFFNESS = strtod(val, &end);
        }
        // dissipation
        else if (strncmp(arg,dissipation_arg,sizeof(dissipation_arg)-1)==0){
            char const* val = arg + (sizeof(dissipation_arg)-1);
            char* end;
            tc.DISSIPATION = strtod(val, &end);
        }
        // time for simulation
        else if (strncmp(arg,final_time_arg,sizeof(final_time_arg)-1)==0){
            char const* val = arg + (sizeof(final_time_arg)-1);
            char* end;
            tc.FINAL_TIME = strtod(val, &end);
        }
        // sine amplitude
        else if (strncmp(arg,amplitude_arg,sizeof(amplitude_arg)-1)==0){
            char const* val = arg + (sizeof(amplitude_arg)-1);
            char* end;
            tc.S_AMPLITUDE = strtod(val, &end);
        }
        else {
            std::cout << "Unknown argument '" << arg << "'" << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    // Create test cases;
    std::vector<testCase> testCases;
    if (argc > 1){
        testCase tc;
        commandLineArguments(tc,argc,argv);
        testCases.push_back(tc);
    } else {
        int testCount = 1;
        for (int i=0; i<testCount; i++){
            testCase tc;
            testCases.push_back(tc);
        }
    }
    // Run and time test cases
    for (int i=0; i<testCases.size(); i++) {
        int runCount = 1;
        try {
            for (int ii=0; ii<runCount; ii++){
                test(testCases[i]);
            }
        }
        catch (const std::exception& ex) {
            std::cout << "Run FAILED due to: " << ex.what() << std::endl;
            return 1;
        }
    }
    return 0;
}

int test(const testCase& tc) {
    using namespace OpenSim;

    auto model = buildWrappingModel(tc);
    model.finalizeConnections();

    // Add table for result processing
    auto table = new TableReporter();
    table->setName("wrapping_results_table");
    table->set_report_time_interval(tc.REPORTING_INTERVAL);
    table->addToReport(model.getComponent("/forceset/spring").getOutput("length"));
    table->addToReport(model.getComponent("/jointset/sliderGround/sliderX").getOutput("value"), "cylinderLocation");
    model.addComponent(table);

    SimTK::State& x0 = model.initSystem();

    // run the simulation
    simulate(model, x0, tc.FINAL_TIME, true);

    // convert spring length table result to vector of doubles
    const auto length = table->getTable().getDependentColumnAtIndex(0);
    const auto position = table->getTable().getDependentColumnAtIndex(1);

//    // output the results
//    std::cout << tc.REPORTING_INTERVAL << std::endl;
//    std::cout << (double)testPass << std::endl;
//    for (const auto &e : anglesTangent){
//        std::cout << e << "\t";
//    }
//    std::cout << std::endl;

    return 0;
}
