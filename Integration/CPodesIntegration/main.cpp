#include <OpenSim/OpenSim.h>
#include <chrono>
#include <vector>

int main(){
    bool visualize = false;

    OpenSim::Model model{"Rajagopal_2015.osim"};
    if (visualize){
        model.setUseVisualizer(true);
    }

    SimTK::State& state = model.initSystem();
    model.equilibrateMuscles(state);

    if (visualize){
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
    }

    for (int i=7; i<8; i++){
//        if (OpenSim::Manager::IntegratorMethod(i) == OpenSim::Manager::IntegratorMethod::CPodes){
//            manager.getIntegrator().setAllowInterpolation(0);
//        }
        std::chrono::milliseconds simTime{0};
        int nSim = 20;
        for (int ii=0; ii<nSim; ii++){
            OpenSim::Manager manager(model);
            manager.setIntegratorMethod(OpenSim::Manager::IntegratorMethod(i));
            manager.initialize(state);

            auto before = std::chrono::high_resolution_clock::now();
            manager.integrate(0.5);
            auto after = std::chrono::high_resolution_clock::now();
            auto dt = after - before;
            simTime += std::chrono::duration_cast<std::chrono::milliseconds>(dt);
        }
        simTime /= nSim;
        std::cout << "simTime("<<i<<"): " << simTime.count() << std::endl;
    }

    return 0;
}
