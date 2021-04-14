#include <vector>
#include <iostream>
#include <chrono>

#include <OpenSim/OpenSim.h>

int main(){
    OpenSim::Model model{"Rajagopal_2015.osim"};
    model.setUseVisualizer(true);

    SimTK::State& st = model.initSystem();
    model.equilibrateMuscles(st);

    // Visualizer settings
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(SimTK::White);

    // Simulation settings
    OpenSim::Manager manager(model);
    manager.setIntegratorAccuracy(3e-1);
    manager.initialize(st);

    // Time the simulation
    auto before = std::chrono::high_resolution_clock::now();
    manager.integrate(1.0);
    auto after = std::chrono::high_resolution_clock::now();
    auto dt = after - before;

    std::chrono::milliseconds simTime =
            std::chrono::duration_cast<std::chrono::milliseconds>(dt);
    std::cout << "\nsimtime: " << simTime.count() << std::endl;

    // Simulation properties
    SimTK::Integrator const& integrator = manager.getIntegrator();
    std::cout << "numStepsAttemped: " <<
                 integrator.getNumStepsAttempted() << std::endl;
    std::cout << "numStepsTaken:    " <<
                 integrator.getNumStepsTaken() << std::endl;
    std::cout << "numIterations:    " <<
                 integrator.getNumIterations() << std::endl;

    return 0;
}
