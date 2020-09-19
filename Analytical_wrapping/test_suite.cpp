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
#include "modelParameters.h"
#include <ctime>
#include <vector>
#include <fstream>

namespace OpenSim {
    Model buildWrappingModel(bool showVisualizer);
}

void run(bool showVisualizer, double finalTime);

int main(int argc, char* argv[]) {
    bool showVisualizer{false};

    try {
        run(showVisualizer, FINAL_TIME);
    }
    catch (const std::exception& ex) {
        std::cout << "Run FAILED due to: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

void run(bool showVisualizer, double finalTime) {
    using namespace OpenSim;

    auto model = buildWrappingModel(showVisualizer);
    //model.printSubcomponentInfo();
    //model.printSubcomponentInfo<Joint>();

    // Add table for results
    std::string sliderLPath{"/jointset/sliderLeft/yCoordSliderLeft"};
    std::string sliderRPath{"/jointset/sliderRight/yCoordSliderRight"};
    auto table = new TableReporter();
    table->setName("wrapping_results_table");
    table->set_report_time_interval(REPORTING_INTERVAL);
    table->addToReport(model.getComponent(sliderLPath).getOutput("value"), "height slider L");
    table->addToReport(model.getComponent(sliderRPath).getOutput("value"), "height slider R");
    table->addToReport(model.getComponent("/forceset/muscle").getOutput("fiber_length"));
    table->addToReport(model.getComponent("/forceset/muscle").getOutput("tendon_length"));
    model.addComponent(table);

    auto console = new ConsoleReporter();
    console->setName("wrapping_results_console");
    console->set_report_time_interval(REPORTING_INTERVAL);
    console->addToReport(model.getComponent(sliderLPath).getOutput("value"), "height slider L");
    console->addToReport(model.getComponent(sliderRPath).getOutput("value"), "height slider R");
    console->addToReport(model.getComponent("/forceset/muscle").getOutput("fiber_length"));
    console->addToReport(model.getComponent("/forceset/muscle").getOutput("tendon_length"));
    model.addComponent(console);

    SimTK::State& x0 = model.initSystem();
    // time the simulation
    clock_t time = clock();
    simulate(model, x0, finalTime,true);
    time = clock() - time;
    std::cout << "Execution time: " <<
                 time << " clicks, " <<
                 (float)time/CLOCKS_PER_SEC << " seconds (sim time = " << finalTime << " seconds)" << std::endl;

    // analyze results
    const auto headings = table->getTable().getColumnLabels();
    const auto leftHeight = table->getTable().getDependentColumnAtIndex(0);
    const auto rightHeight = table->getTable().getDependentColumnAtIndex(1);
    const auto fiberLength = table->getTable().getDependentColumnAtIndex(2);
    const auto tendonLength = table->getTable().getDependentColumnAtIndex(3);

    bool testPass = true;
    for (int i=0; i<FINAL_TIME/REPORTING_INTERVAL; i++){
        double lNumerical = (double) fiberLength[i] + tendonLength[i];
        double s = BODY_SIZE;
        double r = CYLINDER_RADIUS;
        double x = BODY_OFFSET;
        double h = CYLINDER_HEIGHT;

        double lAnalytical, lCylinder;
        if (rightHeight[i]+s/2 >= h+r) {
            lAnalytical = 2*x;
        }
        else {
            double hD = h - (leftHeight[i] + s / 2);
            // distance center cylinder and connection point
            double d = sqrt(pow(x, 2) + pow(hD, 2));
            // length from muscle connection point to tangent circle
            double lTangent = sqrt(pow(d, 2) - pow(r, 2));
            // angle connection point to horizontal
            double beta = atan(r / lTangent) + atan(hD / x);
            // height of extension of connection point to tangent circle (h + small part)
            double H = tan(beta) * x;
            // center of cylinder, horizontal line until it touches muscle
            double y = ((H - hD) / H) * x;
            // angle horizontal and perpendicular to tangent to circle
            double theta = acos(r / y);
            // length over cylinder part
            lCylinder = (SimTK::Pi - 2 * theta) * r;
            // total analytical length
            lAnalytical = 2 * lTangent + lCylinder;
        }

        double margin = 0.01;
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