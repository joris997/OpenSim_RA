#include <vector>
#include <string.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>

#include <OpenSim/OpenSim.h>

#include "tests.h"

int main(int argc, char **argv){
//    OpenSim::Model pbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/Hopper.osim");
//    OpenSim::Model pbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/RajagopalModel/R_left.osim");
    OpenSim::Model pbpModel("/home/none/Documents/cpp/OpenSim/OpenSim_RA/models/Arm26/arm26.osim");
    OpenSim::Model* fbpModel = pbpModel.clone();

    pbpModel.finalizeConnections();
    pbpModel.finalizeFromProperties();
    pbpModel.initSystem();
    pbpModel.printSubcomponentInfo();

    // Create folder for txt file data and new osim file
    std::string modelName = pbpModel.getName();
    std::string dirName = modelName.substr(0,modelName.find(".",0));
    if (mkdir(dirName.c_str(),0777) == -1){
        std::cout << "Folder for model; " << dirName << " already created" << std::endl;
        std::cout << "so I'll just use that one and overwrite everything" << std::endl;
    } else {
        std::cout << "Folder for model; " << dirName << " created" << std::endl;
    }

    // Converting
    std::vector<OpenSim::FunctionBasedPath> fbps;
    std::vector<OpenSim::PointBasedPath> pbps;
    std::ofstream printFile;
    int id = 0;

    for(OpenSim::PathActuator& pa :
        pbpModel.updComponentList<OpenSim::PathActuator>()){
        // const reference to a geometrypath
        auto &pbp = pa.getGeometryPath();

        // dynamic cast to check if its a pointbasedpath
        const OpenSim::PointBasedPath* pbpp = dynamic_cast<const OpenSim::PointBasedPath*>(&pbp);
        pbps.push_back(*pbpp);
        if(pbpp != nullptr){
            std::cout << "pa: " << pa.getName() << std::endl;

            OpenSim::FunctionBasedPath fbp(pbpModel,*pbpp,id);

            printFile.open(dirName+"/FBP"+std::to_string(id)+".xml");
            fbp.printContent(printFile);

            fbps.push_back(fbp);
            id++;
        }
    }

    int cnt = 0;
    const OpenSim::PointBasedPath* pbpp;
    std::vector<OpenSim::PathActuator*> pap;
    for(OpenSim::PathActuator& pa :
        fbpModel->updComponentList<OpenSim::PathActuator>()){
        pap.push_back(&pa);
    }
    for (unsigned i=0; i<pap.size(); i++){
        auto &pbp = pap[i]->getGeometryPath();
        pbpp = dynamic_cast<const OpenSim::PointBasedPath*>(&pbp);
        if(pbpp != nullptr){
            pap[i]->updProperty_GeometryPath().setValue(fbps[cnt]);
            cnt++;
        }
    }
    fbpModel->finalizeConnections();
    fbpModel->finalizeFromProperties();
    fbpModel->initSystem();
    fbpModel->printSubcomponentInfo();



    // ADD REPORTER
    if (false){
        auto pbpReporter = new OpenSim::ConsoleReporter();
        pbpReporter->setName("pbp_results");
        pbpReporter->set_report_time_interval(0.1);
        pbpReporter->addToReport(pbpModel.getComponent("/jointset/slider/yCoord").getOutput("value"), "height");
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/TRIlong/pointbasedpath").getOutput("length"));
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/TRIlat/pointbasedpath").getOutput("length"));
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/TRImed/pointbasedpath").getOutput("length"));
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/BIClong/pointbasedpath").getOutput("length"));
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/BICshort/pointbasedpath").getOutput("length"));
//        pbpReporter->addToReport(pbpModel.getComponent("/forceset/BRA/pointbasedpath").getOutput("length"));
        pbpModel.addComponent(pbpReporter);

        auto fbpReporter = new OpenSim::ConsoleReporter();
        fbpReporter->setName("fbp_results");
        fbpReporter->set_report_time_interval(0.1);
        fbpReporter->addToReport(fbpModel->getComponent("/jointset/slider/yCoord").getOutput("value"), "height");
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/TRIlong/functionbasedpath").getOutput("length"));
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/TRIlat/functionbasedpath").getOutput("length"));
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/TRImed/functionbasedpath").getOutput("length"));
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/BIClong/functionbasedpath").getOutput("length"));
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/BICshort/functionbasedpath").getOutput("length"));
//        fbpReporter->addToReport(fbpModel->getComponent("/forceset/BRA/functionbasedpath").getOutput("length"));
        fbpModel->addComponent(fbpReporter);
    }



    // PRINT MODEL TO FILE
//    fbpModel->print("HopperFBP.osim");
//    fbpModel->print("Rajagopal2015FBP.osim");
    fbpModel->print("Arm26FBP.osim");

//    freopen("consoleOut.txt","w",stdout);

//    SimTK::State& pbpSt = pbpModel.initSystem();
//    pbpModel.setUseVisualizer(true);
//    pbpModel.updMatterSubsystem().setShowDefaultGeometry(true);
//    OpenSim::Manager pbpMmanager(pbpModel);
//    pbpMmanager.initialize(pbpSt);
//    pbpMmanager.integrate(3.5);

//    SimTK::State& fbpSt = fbpModel->initSystem();
//    fbpModel->setUseVisualizer(true);
//    fbpModel->updMatterSubsystem().setShowDefaultGeometry(true);
//    OpenSim::Manager fbpManager(*fbpModel);
//    fbpManager.initialize(fbpSt);
//    fbpManager.integrate(3.5);


//    simulateFbpModel(fbpModel);
//    simulateBoth(pbpModel,fbpModel);
//    testAccuracy(pbpModel,fbpModel,fbps);
    testPerformance(pbpModel,fbpModel);

    return 0;
}







//    // Argument parsing
//    if(argc == 0){
//        std::cout << "provide at least an .osim model as argument" << std::endl;
//        return 1;
//    }
//    OpenSim::Model model;
//    if(argv[0] == "--help"){
//        std::cout << "help for Interpolation conversion tool" << std::endl;
//        return 0;
//    } else {
//        try {
////            model({argv[0]});
//        } catch (const std::exception& e){
//            std::cout << e.what() << std::endl;
//            return 2;
//        }
//    }



//    while (cnt < fbps.size()){
//        for(OpenSim::PathActuator& pa :
//            fbpModel->updComponentList<OpenSim::PathActuator>()){

//            auto &pbp = pa.getGeometryPath();
//            pbpp = dynamic_cast<const OpenSim::PointBasedPath*>(&pbp);
//            if(pbpp != nullptr){
//                pa.updProperty_GeometryPath().setValue(fbps[cnt]);
//                cnt++;
//                break;
//            }
//        }
//    }
