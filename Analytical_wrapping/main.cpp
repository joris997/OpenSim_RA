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

std::ofstream outputFile("../Analytical_wrapping/path_point_plotting/outputFile.txt");

using OpenSim::Exception;
using OpenSim::Model;
using OpenSim::ConsoleReporter;
using SimTK::Vec3;
using OpenSim::Exception;

//Model buildWrappingModel(const testCase& tc);
//Model buildWrappingModelPathPoints(const testCase& tc, bool moving);
//Model buildWrappingModelHorizontal(const testCase& tc);
Model buildWrappingModelDouble(const testCase& tc);

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

bool checkAnalytical(const OpenSim::TableReporter_<double,double>& table, const testCase& tc){
    // unpack the table to analyze the results
    const auto headings = table.getTable().getColumnLabels();
    const auto leftHeight = table.getTable().getDependentColumnAtIndex(0);
    const auto rightHeight = table.getTable().getDependentColumnAtIndex(1);
    const auto fiberLength = table.getTable().getDependentColumnAtIndex(2);
    const auto tendonLength = table.getTable().getDependentColumnAtIndex(3);

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
    return testPass;
}

double test(const testCase& tc);

int main(int argc, char* argv[]) {
    // Create test cases;
    std::vector<testCase> testCases(20);
    for (int i=0; i<testCases.size(); i++){
//        testCases[i].DISCRETIZATION = 3 + i*5;
//        testCases[i].OPT_FIBER_LENGTH += 0.025*i;
        testCases[i].TENDON_SLACK_LENGTH += 0.025*i;
    }

    double runTimes[testCases.size()];
    for (int i=0; i<testCases.size(); i++) {
        int runCount = 1;
        try {
            for (int ii=0; ii<runCount; ii++){
                runTimes[i] += test(testCases[i]);
            }
        }
        catch (const std::exception& ex) {
            std::cout << "Run FAILED due to: " << ex.what() << std::endl;
            return 1;
        }
        runTimes[i] = runTimes[i]/((double)runCount+1);
    }
    return 0;
}

double test(const testCase& tc) {
    using namespace OpenSim;

//    auto model = buildWrappingModelHorizontal(tc);
//    auto model = buildWrappingModelPathPoints(tc, false);
//    auto model = buildWrappingModel(tc);
    auto model = buildWrappingModelDouble(tc);

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
    simulate(model, x0, tc.FINAL_TIME, true);
    ticks = clock() - ticks;
    double runTime = (float)ticks/CLOCKS_PER_SEC;
    std::cout << "Execution time: " <<
              ticks << " clicks, " <<
              runTime << " seconds (sim time = " << tc.FINAL_TIME << " seconds)" << std::endl;

    // compute the muscle length numerically
    const auto fiberLength = table->getTable().getDependentColumnAtIndex(2);
    const auto tendonLength = table->getTable().getDependentColumnAtIndex(3);
    std::vector<double> muscleLength(fiberLength.size());
    std::vector<double> fiberLengthVector(fiberLength.size());
    std::vector<double> tendonLengthVector(tendonLength.size());
    for(int i=0; i<tc.FINAL_TIME/tc.REPORTING_INTERVAL; i++){
        muscleLength[i] = (double)fiberLength[i] + (double)tendonLength[i];
        fiberLengthVector[i] = (double)fiberLength[i];
        tendonLengthVector[i] = (double)tendonLength[i];
    }

    // compare analytical and numerical solution
    if (false){
        if (checkAnalytical(*table,tc)){
            std::cout << "Test status (" << tc.CYLINDER_ROT[1] << "): [PASSED]" << std::endl;
        } else {
            std::cout << "Test status: [FAILED]" << std::endl;
        }
    }
    
    // write results to a text file
    if (true){
        outputFile << runTime << "\n";
        outputFile << tc.DISCRETIZATION << "\n";
        outputFile << tc.REPORTING_INTERVAL << "\n";

        outputFile << tc.MUSCLE_MAX_FORCE << "\n";
        outputFile << tc.OPT_FIBER_LENGTH << "\n";
        outputFile << tc.TENDON_SLACK_LENGTH << "\n";

        for (const auto &e : muscleLength) outputFile << e << "\t";
        outputFile << "\n";
        for (const auto &e : fiberLengthVector) outputFile << e << "\t";
        outputFile << "\n";
        for (const auto &e : tendonLengthVector) outputFile << e << "\t";
        outputFile << "\n";
    }
    return runTime;
}





//    model.printSubcomponentInfo();
//    model.printSubcomponentInfo<Joint>();
//    model.finalizeConnections();
//    model.print("model.osim");
//    addConsole(model, tc);



// this gets recreated every function call
//std::vector<int> results;
//results.clear();
//
//for (int i=0; i<100; ++i){
//results.push_back(i);
//}
// improve:
//static std::vector<int> results;
//results.clear();
//
//for (int i=0; i<100; ++i){
//    results.push_back(i);
//}
// here using static runs the risk of crashing when two function calls to the same function
//thread_local std::vector<int> results // which is now thread-safe
