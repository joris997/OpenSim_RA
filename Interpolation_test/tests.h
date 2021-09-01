#include <OpenSim/OpenSim.h>

void simulateFbpModel(OpenSim::Model* fbpModel){
    SimTK::State& fbpSt = fbpModel->initSystem();

    fbpModel->setUseVisualizer(true);
    fbpModel->updMatterSubsystem().setShowDefaultGeometry(true);

//    simulate(*fbpModel,fbpSt,5);
    OpenSim::Manager manager(*fbpModel);
    manager.initialize(fbpSt);
    manager.integrate(5);
}



void simulateBoth(OpenSim::Model pbpModel, OpenSim::Model* fbpModel){
    SimTK::State& pbpSt = pbpModel.initSystem();
    SimTK::State& fbpSt = fbpModel->initSystem();

    OpenSim::Manager pbpMmanager(pbpModel);
    pbpMmanager.initialize(pbpSt);
    pbpMmanager.integrate(5);

    OpenSim::Manager fbpManager(*fbpModel);
    fbpManager.initialize(fbpSt);
    fbpManager.integrate(5);
}



//void testAccuracy(OpenSim::Model pbpModel, OpenSim::Model* fbpModel, std::vector<OpenSim::FunctionBasedPath> fbps){
//    Interpolate interp = fbps[0].getInterpolate();

//    SimTK::State& pbpSt = pbpModel.initSystem();
//    SimTK::State& fbpSt = fbpModel->initSystem();

//    fbpModel->equilibrateMuscles(fbpSt);
//    fbpModel->realizeVelocity(fbpSt);
//    const OpenSim::PointBasedPath* pbpp2;
//    for(OpenSim::PathActuator& pa :
//        fbpModel->updComponentList<OpenSim::PathActuator>()){
//        auto &pbp = pa.getGeometryPath();

//        pbpp2 = dynamic_cast<const OpenSim::PointBasedPath*>(&pbp);
//    }

//    std::vector<const OpenSim::Coordinate *> coords;
//    std::vector<const OpenSim::Coordinate *> affectingCoords;
//    Nonzero_conditions cond;
//    for (OpenSim::Coordinate const& c : fbpModel->getComponentList<OpenSim::Coordinate>()){
//        if (c.getMotionType() != OpenSim::Coordinate::MotionType::Coupled){
//            coords.push_back(&c);
//        }
//    }
//    for (OpenSim::Coordinate const* c : coords){
//        if (coord_affects_muscle(*pbpp2,*c,fbpSt,cond)){
//            affectingCoords.push_back(c);
//            std::cout << "affecting coord: " << c->getName() << std::endl;
//        }
//    }
//    for (int i=0; i<affectingCoords.size(); i++){
//        affectingCoords[i]->setValue(fbpSt,0);
//    }
//    std::cout << "test2, getLength fbp: " << interp.getLength(fbpSt) << std::endl;
////    std::cout << "test2, getLength pbp: " << pbpp->getLength(st2) << "\n" << std::endl;
//}



void testPerformance(OpenSim::Model pbpModel, OpenSim::Model fbpModel, bool visualize=false){
    // TESTING PERFORMANCE
    if (visualize){
        pbpModel.setUseVisualizer(true);
        fbpModel.setUseVisualizer(true);
    }

    SimTK::State& pbpSt = pbpModel.initSystem();
    SimTK::State& fbpSt = fbpModel.initSystem();

    if (visualize){
        pbpModel.updMatterSubsystem().setShowDefaultGeometry(true);
        fbpModel.updMatterSubsystem().setShowDefaultGeometry(true);
    }


    std::chrono::milliseconds pbpSimTime{0};
    std::chrono::milliseconds fbpSimTime{0};
    int pbpStepsAttempted = 0;
    int fbpStepsAttempted = 0;
    int pbpStepsTaken = 0;
    int fbpStepsTaken= 0;

    int n = 1;
    double tFinal = 1.0;
    OpenSim::Storage pbpStates, fbpStates;
    for (int i=0; i<n; i++){
        OpenSim::Manager manager(pbpModel);
        manager.initialize(pbpSt);
        manager.setWriteToStorage(true);
        auto before = std::chrono::high_resolution_clock::now();
        manager.integrate(tFinal);
        auto after = std::chrono::high_resolution_clock::now();
        auto dt = after - before;
        pbpStepsAttempted += manager.getIntegrator().getNumStepsAttempted();
        pbpStepsTaken += manager.getIntegrator().getNumStepsTaken();
        pbpSimTime += std::chrono::duration_cast<std::chrono::milliseconds>(dt);
        std::cout << "pbp " << i << std::endl;

        manager.getStateStorage().print("pbpStates.sto");
    }

    for (int i=0; i<n; i++){
        OpenSim::Manager manager(fbpModel);
        manager.initialize(fbpSt);
        manager.setWriteToStorage(true);
        auto before = std::chrono::high_resolution_clock::now();
        manager.integrate(tFinal);
        auto after = std::chrono::high_resolution_clock::now();
        auto dt = after - before;
        fbpStepsAttempted += manager.getIntegrator().getNumStepsAttempted();
        fbpStepsTaken += manager.getIntegrator().getNumStepsTaken();
        fbpSimTime += std::chrono::duration_cast<std::chrono::milliseconds>(dt);
        std::cout << "fbp " << i << std::endl;

        manager.getStateStorage().print("fbpStates.sto");
    }

    pbpSimTime /= n;
    fbpSimTime /= n;
    pbpStepsAttempted /= n;
    fbpStepsAttempted /= n;
    pbpStepsTaken /= n;
    fbpStepsTaken /= n;

    double pbpTime = (double)pbpSimTime.count();
    double fbpTime = (double)fbpSimTime.count();
    std::cout << "\navg-time PBP:        " << pbpSimTime.count() << std::endl;
    std::cout << "avg-time FBP:        "   << fbpSimTime.count() << std::endl;

    std::cout << "\nsteps attempted PBP: " << pbpStepsAttempted << std::endl;
    std::cout << "steps attempted FBP: "   << fbpStepsAttempted << std::endl;

    std::cout << "\nsteps taken PBP:     " << pbpStepsTaken << std::endl;
    std::cout << "steps taken FBP:     "   << fbpStepsTaken << std::endl;

    std::cout << "\nwe are " << pbpTime/fbpTime << " times as fast!" << std::endl;
}


void testAccuracy(OpenSim::Model pbpModel, OpenSim::Model fbpModel){
    SimTK::State& pbpSt = pbpModel.initSystem();
    SimTK::State& fbpSt = fbpModel.initSystem();

    // get pointbasedpaths of pbpModel
    for (OpenSim::PointBasedPath const& pbp : pbpModel.getComponentList<OpenSim::PointBasedPath>()){
        std::cout << "name:   " << pbp.getName() << std::endl;
        std::cout << "length: " << pbp.getLength(pbpSt) << std::endl;
//        std::cout << "speed:  " << pbp.getLengtheningSpeed(pbpSt) << std::endl;
    }
    // get functionbasedpaths of fbpModel
    for (OpenSim::FunctionBasedPath const& fbp : fbpModel.getComponentList<OpenSim::FunctionBasedPath>()){
        std::cout << "name:   " << fbp.getName() << std::endl;
        std::cout << "length: " << fbp.getLength(fbpSt) << std::endl;
        std::cout << "speed:  " << fbp.getLengtheningSpeed(fbpSt) << std::endl;
    }

}
