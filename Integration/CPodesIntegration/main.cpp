#include <OpenSim/OpenSim.h>
#include <chrono>
#include <vector>
#include <fstream>

int main(){
    bool visualize = false;

//    OpenSim::Model model{"arm26.osim"};
    std::vector<std::string> integrators;
    integrators.push_back("ExplicitEuler");
    integrators.push_back("RungeKutta2");
    integrators.push_back("RungeKutta3");
    integrators.push_back("RungeKuttaFeldberg");
    integrators.push_back("RungeKuttaMerson");
    integrators.push_back("SemiExplicitEuler2");
    integrators.push_back("Verlet");

    std::vector<std::string> models;
    models.push_back("Rajagopal_2015.osim");
    models.push_back("arm26.osim");
    models.push_back("gait10dof18musc.osim");
    models.push_back("gait2354_simbody.osim");
    models.push_back("gait2392_millard2012muscle.osim");
    models.push_back("gait2392_thelen2003muscle.osim");
    models.push_back("leg39.osim");
    models.push_back("SoccerKickingModel.osim");
    models.push_back("Tug_of_War.osim");
    models.push_back("ToyLandingModel.osim");

    std::ofstream myfile;
    myfile.open("solver_results.txt");
    for (auto& modeli : models){
        OpenSim::Model model{modeli};
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

        myfile << "\n" << modeli << "\n";
        for (int i=0; i<integrators.size(); i++){
    //        if (OpenSim::Manager::IntegratorMethod(i) == OpenSim::Manager::IntegratorMethod::CPodes){
    //            manager.getIntegrator().setAllowInterpolation(0);
    //            manager.getIntegrator().setUseCPodesProjection();
    //        }
            std::chrono::milliseconds simTime{0};
            int nSim = 4;
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
            myfile << "simTime("<<i<<"): " << simTime.count() << "\t"
                   << integrators[i] << "\n";
//            std::cout << "simTime("<<i<<"): " << simTime.count() << std::endl;
        }
    }
    myfile.close();
    return 0;
}
