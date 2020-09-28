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
#include <array>

static const std::string sliderLPath{"/jointset/sliderLeft/yCoordSliderLeft"};
static const std::string sliderRPath{"/jointset/sliderRight/yCoordSliderRight"};

using OpenSim::Exception;
using OpenSim::Model;
using OpenSim::ConsoleReporter;
using SimTK::Vec3;
using fractional_secs = std::chrono::duration<double, std::ratio<1>>;

Model buildWrappingModel(bool showVisualizer, const testCase& tc);

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

fractional_secs test(const testCase& tc) {
    using OpenSim::TableReporter;

    auto model = buildWrappingModel(tc.SHOW_VISUALIZER, tc);
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

    auto t0 = std::chrono::high_resolution_clock::now();
    simulate(model, x0, tc.FINAL_TIME, true);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = fractional_secs(t1 - t0);

    std::cout << "Execution time: " << dt.count() << " seconds (sim time = " << tc.FINAL_TIME << " seconds)" << std::endl;

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

    return dt;
}

int main(int argc, char* argv[]) {
    if (argc > 1) {
        // If the caller provides arguments, those arguments are assumed to be
        // real numbers that represent the cylinder's Y rotation. Test each
        // rotation in series.

        for (int i = 1; i < argc; ++i) {
            testCase tc{};
            double rot = std::stod(argv[i]);
            std::cerr << argv[0] << ": " << "attempting cylinder rotation of " << rot << " radians " << std::endl;
            tc.CYLINDER_ROT[1] = rot;
            test(tc);
        }

        // do not run the rest of the test suite if the caller provided an arg
        return 0;
    }

    // number of repeats for computing the average run-time
    constexpr size_t runCount = 6;

    // create test cases
    constexpr size_t numTests = 6;
    std::array<fractional_secs, numTests> runTimes{};
    std::array<testCase, numTests> tests{};

    tests[0].CYLINDER_ROT = Vec3(0.0,0.0,0.0);
    tests[1].CYLINDER_ROT = Vec3(0.0,0.2,0.0);
    tests[2].CYLINDER_ROT = Vec3(0.0,0.4,0.0);
    tests[3].CYLINDER_ROT = Vec3(0.0,0.6,0.0);
    tests[4].CYLINDER_ROT = Vec3(0.0,0.8,0.0);
    tests[5].CYLINDER_ROT = Vec3(0.0,1.0,0.0);

    std::cout << tests.size() << std::endl;

    // run each test
    for (size_t i = 0; i < tests.size(); ++i) {
        fractional_secs rollingAvg{0};
        for (size_t j = 0; j < runCount; ++j) {
            fractional_secs t = test(tests[i]);
            rollingAvg = (j*rollingAvg + t)/(j + 1);
        }
        runTimes[i] = rollingAvg;
    }

    for (size_t i = 0; i < tests.size(); i++) {
        std::cout << "Average test (" << tests[i].CYLINDER_ROT[1] << "): " << runTimes[i].count() << std::endl;
    }

    return 0;
}
