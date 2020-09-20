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

void test(const testCase& tc);

int main(int argc, char* argv[]) {
    // Create test cases;
    testCase a;
    testCase b;
    // Run each test case
    for (testCase const& tc: {a}) {
        try {
            test(tc);
        }
        catch (const std::exception& ex) {
            std::cout << "Run FAILED due to: " << ex.what() << std::endl;
            return 1;
        }
    }
    return 0;
}

void test(const testCase& tc) {
    using namespace OpenSim;

    auto model = buildWrappingModel(tc.SHOW_VISUALIZER, tc);
    //model.printSubcomponentInfo();
    //model.printSubcomponentInfo<Joint>();

    // Add console for results
    addConsole(model, tc);
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
    clock_t time = clock();
    simulate(model, x0, tc.FINAL_TIME,true);
    time = clock() - time;
    std::cout << "Execution time: " <<
                 time << " clicks, " <<
                 (float)time/CLOCKS_PER_SEC << " seconds (sim time = " << tc.FINAL_TIME << " seconds)" << std::endl;

    // unpack the table to analyze the results
    const auto headings = table->getTable().getColumnLabels();
    const auto leftHeight = table->getTable().getDependentColumnAtIndex(0);
    const auto rightHeight = table->getTable().getDependentColumnAtIndex(1);
    const auto fiberLength = table->getTable().getDependentColumnAtIndex(2);
    const auto tendonLength = table->getTable().getDependentColumnAtIndex(3);

    bool testPass = true;
    for (int i=0; i<tc.FINAL_TIME/tc.REPORTING_INTERVAL; i++){
        double lNumerical  = fiberLength[i] + tendonLength[i];
        double lAnalytical = analyticalSolution(leftHeight[i], tc);

        double margin = 0.01;   // 1% margin allowed
        if (lNumerical > (1+margin)*lAnalytical || lNumerical < (1-margin)*lAnalytical) {
            testPass = false;
            std::cout << "[WARNING] Numerical does not correspond to Analytical at index: " << i << std::endl;
        }
    }
    if (testPass){
        std::cout << "Test status: [PASSED]" << std::endl;
    } else {
        std::cout << "Test status: [FAILED]" << std::endl;
    }
}