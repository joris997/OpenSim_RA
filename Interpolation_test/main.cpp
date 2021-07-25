#include <vector>
#include <string.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>

#include <OpenSim/OpenSim.h>

#include "tests.h"

using namespace OpenSim;

void testParser(const std::string &path);
struct Discretization final {
    double begin;
    double end;
    int nPoints;
    double gridsize;
};

int main(int argc, char **argv){
//    Model pbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/Hopper.osim");
//    FunctionBasedPathConversionTool tool("/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/Hopper.osim", "Hopper_FBP.osim");

//    std::string absolutePath = "/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/Arm26/arm26.osim";
//    Model pbpModel(absolutePath);
//    FunctionBasedPathConversionTool tool(absolutePath, "arm26_FBP");

//    tool.run();

    std::string absolutePath = "/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/RajagopalModel/Rajagopal2015.osim";
    Model pbpModel(absolutePath);
    FunctionBasedPathConversionTool tool(absolutePath, "Rajagopal2015_FBP");

    tool.run();

//    Model fbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/build/Hopper_FBP.osim.osim");
//    Model fbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/build/arm26_FBP.osim");
    Model fbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/build/Rajagopal2015_FBP.osim");

    pbpModel.initSystem();
    fbpModel.initSystem();

//    auto pbpReporter = new ConsoleReporter();
//    pbpReporter->setName("pbp_results");
//    pbpReporter->set_report_time_interval(0.01);
//    pbpReporter->addToReport(pbpModel.getComponent("/forceset/TRIlong/pointbasedpath").getOutput("length"));
//    pbpReporter->addToReport(pbpModel.getComponent("/forceset/TRIlong/pointbasedpath").getOutput("lengthening_speed"));
//    pbpModel.addComponent(pbpReporter);

//    auto fbpReporter = new ConsoleReporter();
//    fbpReporter->setName("fbp_results");
//    fbpReporter->set_report_time_interval(0.01);
//    fbpReporter->addToReport(fbpModel.getComponent("/forceset/TRIlong/functionbasedpath").getOutput("length"));
//    fbpReporter->addToReport(fbpModel.getComponent("/forceset/TRIlong/functionbasedpath").getOutput("lengthening_speed"));
//    fbpReporter->addToReport(fbpModel.getComponent("jointset/r_shoulder/r_shoulder_elev").getOutput("value"));
//    fbpReporter->addToReport(fbpModel.getComponent("jointset/r_elbow/r_elbow_flex").getOutput("value"));
//    fbpModel.addComponent(fbpReporter);


    pbpModel.initSystem();
    pbpModel.printSubcomponentInfo();

    // fromP and only then connections()!!!
    fbpModel.initSystem();
    fbpModel.printSubcomponentInfo();


    // print file
//    int cnt = 1;
//    for (FunctionBasedPath const& fbp : fbpModel.getComponentList<FunctionBasedPath>()){
//        std::string path = "/home/none/Documents/cpp/OpenSim/OpenSim_RA/build/arm26_FBP.osim_DATA_"+ std::to_string(cnt) +"_copy.txt";
//        std::ofstream ofName{path};
//        fbp.printContent(ofName);
//        cnt++;
//    }

    testPerformance(pbpModel,fbpModel);
//    testAccuracy(pbpModel,fbpModel);

    return 0;
}
