/* -------------------------------------------------------------------------- *
 *                     OpenSim:  exampleHopperDevice.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Changes made by: Joris Verhagen                                            *
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

/* This example demonstrates some of the new features of the OpenSim 4.0 API.
The Component architecture allows us to join sub-assemblies to form larger
Models, with information flowing between Components via Inputs, Outputs, and
Sockets. For more information, please refer to the Component documentation.*/

#include <OpenSim/OpenSim.h>
#include <ctime>
#include <vector>
#include <fstream>

static const double REPORTING_INTERVAL{ 0.2 };
static const double FINAL_TIME{ 5.0 };

// Provide the name of the coordinate corresponding to the hopper's height.
// Hint: the hopper's pelvis is attached to ground with a vertical slider
// joint; see buildHopperModel.cpp and Component::printOutputInfo().
static const std::string hopperHeightCoord{"/jointset/slider/yCoord"};


namespace OpenSim {

// Forward declarations for methods used below.
    Model buildHopper(bool showVisualizer, const std::string& patella_shape);    //defined in buildHopperModel.cpp

    void addConsoleReporterToHopper(Model& hopper)
    {
        // Create a new ConsoleReporter. Set its name and reporting interval.
        auto reporter = new ConsoleReporter();
        reporter->setName("hopper_results");
        reporter->set_report_time_interval(REPORTING_INTERVAL);

        // Connect outputs from the hopper to the reporter's inputs. Try
        // reporting the hopper's height, the vastus muscle's activation, the
        // knee angle, and any other variables of interest.
        reporter->addToReport(
                hopper.getComponent(hopperHeightCoord).getOutput("value"), "height");

        reporter->addToReport(
                hopper.getComponent("/forceset/vastus").getOutput("activation"));

        reporter->addToReport(
                hopper.getComponent("/jointset/knee/kneeFlexion").
                        getOutput("value"), "knee_angle");

        // Add the reporter to the model.
        hopper.addComponent(reporter);
    }
} // namespace OpenSim


void run(bool showVisualizer, double finalTime);


int main(int argc, char* argv[]) {
//    // Suppress/show visualizer.
//    bool showVisualizer{true};

//    for(int i = 0; i < argc; ++i)
//        if(strcmp(argv[i], "noVisualizer") == 0)
//            showVisualizer = false;

//    try {
//        run(showVisualizer, FINAL_TIME);
//    }
//    catch (const std::exception& ex) {
//        std::cout << "Hopper Example Failed to run due to the following Exception: "
//                  << ex.what() << std::endl;
//        return 1;
//    }
//    return 0;
    std::string wrap_surface = "cylinder";
    OpenSim::Model hopper = OpenSim::buildHopper(true,wrap_surface);
    hopper.finalizeConnections();
    hopper.print("Hopper.osim");

}


void run(bool showVisualizer, double finalTime)
{
    using namespace OpenSim;
    // overwrite visualizer boolean for debugging
    showVisualizer = true;
    std::string wrap_surfaces[1] = {"cylinder"};
//    std::string wrap_surfaces[4] = {"cylinder", "ellipsoid", "sphere", "torus"};
//    std::string wrap_surfaces[6] = {"cylinder", "ellipsoid", "sphere", "torus", "points","blank"};

    int n_surfaces = sizeof(wrap_surfaces)/sizeof(wrap_surfaces[0]);
    // create an array of vectors containing the time of each run for each wrapping surface
    std::vector<double> results[n_surfaces];
    // create an array of doubles containing the average time for each wrapping surface
    float avg_results[n_surfaces];

    // loop through each wrapping surface
    for (int i=0; i<n_surfaces; i++){
        // somehow have to set this manually to 0.0, otherwise infinitely large init values
        avg_results[i] = 0.0;
        // build the hopper with the right patella shape
        auto patella_shape = wrap_surfaces[i];
        auto hopper = buildHopper(showVisualizer,patella_shape);
        // and save as an .osim file
        hopper.finalizeConnections();
        hopper.print("Hopper.osim");

        addConsoleReporterToHopper(hopper);
        // number of simulations run per patella shape
        int sim_num = 1;
        for (int ii=1; ii<sim_num+1; ii++){
            SimTK::State& sHop = hopper.initSystem();
            // time the simulation time of each patella shape
            clock_t time = clock();
            simulate(hopper, sHop, finalTime);
            time = clock() - time;
            std::cout << patella_shape << " execution time [" << ii << "]: " <<
                      time << " clicks, " <<
                      (float)time/CLOCKS_PER_SEC << " seconds" << std::endl;
            results[i].push_back(time);
            avg_results[i] += (float)time/CLOCKS_PER_SEC;
        }
        // obtain the average simulation time for each patella shape
        avg_results[i] = avg_results[i]/(float)(sim_num);
    }
    auto tool = AnalyzeTool();
    // writing the results to the console as a summary
    for (int i=0; i<n_surfaces; i++){
        std::cout << "Avg time " << wrap_surfaces[i] << "\t" << avg_results[i] << "\n";
    }
};

//            hopper.printSubcomponentInfo();
//            hopper.printSubcomponentInfo<Joint>();
//            hopper.getComponent("/bodyset/thigh").printOutputInfo(false);
//            addConsoleReporterToHopper(hopper);
