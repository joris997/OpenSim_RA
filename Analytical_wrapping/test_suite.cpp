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
#include "analyticalSolution.h"
#include <ctime>
#include <vector>
#include <fstream>

static const std::string sliderLPath{"/jointset/sliderLeft/yCoordSliderLeft"};
static const std::string sliderRPath{"/jointset/sliderRight/yCoordSliderRight"};

using namespace OpenSim;
using namespace SimTK;
using OpenSim::Exception;

Model buildWrappingModel(bool showVisualizer, const testCase& tc);

Model buildWrappingModelPathPoints(bool showVisualizer, const testCase& tc);

void addConsole(Model& model, const testCase& tc){
    auto console = new ConsoleReporter();
    console->setName("wrapping_results_console");
    console->set_report_time_interval(tc.REPORTING_INTERVAL);
    console->addToReport(model.getComponent(sliderLPath).getOutput("value"), "height slider L");
    console->addToReport(model.getComponent(sliderRPath).getOutput("value"), "height slider R");
    console->addToReport(model.getComponent("/forceset/muscle").getOutput("fiber_length"));
    console->addToReport(model.getComponent("/forceset/muscle").getOutput("tendon_length"));
    model.addComponent(console);
}

double test(const testCase& tc);

int main(int argc, char* argv[]) {
    // Create test cases;
    testCase a;
    testCase b;
    testCase c;
    testCase d;
    testCase e;
    testCase f;
    b.CYLINDER_ROT = Vec3(0.0,0.2,0.0);
    c.CYLINDER_ROT = Vec3(0.0,0.4,0.0);
    d.CYLINDER_ROT = Vec3(0.0,0.6,0.0);
    e.CYLINDER_ROT = Vec3(0.0,0.8,0.0);
    f.CYLINDER_ROT = Vec3(0.0,1.0,0.0);

    // Run each test case
//    testCase tests[6] = {a,b,c,d,e,f};
    testCase tests[1] = {a};
    int testCount = sizeof(tests)/sizeof(tests[0]);
    double runTimes[testCount];
    for (int i=0; i<testCount; i++) {
        int runCount = 1;
        try {
            for (int ii=0; ii<runCount; ii++){
                runTimes[i] += test(tests[i]);
            }
        }
        catch (const std::exception& ex) {
            std::cout << "Run FAILED due to: " << ex.what() << std::endl;
            return 1;
        }
        runTimes[i] = runTimes[i]/((double)runCount+1);
    }
    for (int i=0; i<testCount; i++){
        std::cout << "Average test (" << tests[i].CYLINDER_ROT[1] << "): " << runTimes[i] << std::endl;
    }
    return 0;
}

double test(const testCase& tc) {
    using namespace OpenSim;

    auto model = buildWrappingModel(tc.SHOW_VISUALIZER, tc);
//    auto model = buildWrappingModelPathPoints(tc.SHOW_VISUALIZER, tc);
    //model.printSubcomponentInfo();
    //model.printSubcomponentInfo<Joint>();

    // Add console for results
//    addConsole(model, tc);
    // Add table for result processing
    auto table = new TableReporter();
    table->setName("wrapping_results_table");
    table->set_report_time_interval(tc.REPORTING_INTERVAL);
    table->addToReport(model.getComponent(sliderLPath).getOutput("value"), "height slider L");
    table->addToReport(model.getComponent(sliderRPath).getOutput("value"), "height slider R");
    table->addToReport(model.getComponent("/forceset/muscle").getOutput("fiber_length"));
    table->addToReport(model.getComponent("/forceset/muscle").getOutput("tendon_length"));
    model.addComponent(table);

    SimTK::State& x0 = model.initSystem();
    // time the simulation
    clock_t ticks = clock();
    simulate(model, x0, tc.FINAL_TIME,true);
    ticks = clock() - ticks;
    double runTime = (float)ticks/CLOCKS_PER_SEC;
    std::cout << "Execution time: " <<
                 ticks << " clicks, " <<
                 runTime << " seconds (sim time = " << tc.FINAL_TIME << " seconds)" << std::endl;

    // unpack the table to analyze the results
    const auto headings = table->getTable().getColumnLabels();
    const auto leftHeight = table->getTable().getDependentColumnAtIndex(0);
    const auto rightHeight = table->getTable().getDependentColumnAtIndex(1);
    const auto fiberLength = table->getTable().getDependentColumnAtIndex(2);
    const auto tendonLength = table->getTable().getDependentColumnAtIndex(3);

    bool testPass = true;
    for (int i=0; i<tc.FINAL_TIME/tc.REPORTING_INTERVAL; i++){
        double lNumerical  = fiberLength[i] + tendonLength[i];
        double lAnalytical = analyticalSolution(leftHeight[i], tc, true);

        double margin = 0.01;   // 0.1% margin allowed
        if (lNumerical > (1+margin)*lAnalytical || lNumerical < (1-margin)*lAnalytical) {
            testPass = false;
            std::cout << "[WARNING] Numerical does not correspond to Analytical at index: " << i << std::endl;
        }
    }
    if (testPass){
        std::cout << "Test status (" << tc.CYLINDER_ROT[1] << "): [PASSED]" << std::endl;
    } else {
        std::cout << "Test status: [FAILED]" << std::endl;
    }
    return runTime;
}