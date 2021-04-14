#include <OpenSim/OpenSim.h>
#include <chrono>
#include <vector>
#include <fstream>

int main(){
    bool visualize = false;
    int nSim = 5;

    std::string pathToModels = "/home/none/Documents/cpp/OpenSim/OpenSim_RA/models";
    std::string pathToMain = "/home/none/Documents/cpp/OpenSim/OpenSim_RA/Integration/ForwardTool";

    std::vector<std::string> integrators;
    integrators.push_back("ExplicitEuler");
    integrators.push_back("RungeKutta2");
    integrators.push_back("RungeKutta3");
    integrators.push_back("RungeKuttaFeldberg");
    integrators.push_back("RungeKuttaMerson");
    integrators.push_back("SemiExplicitEuler2");
    integrators.push_back("Verlet");

    std::vector<std::string> models;
    models.push_back(pathToModels + "/ArmSwing/arm26_Setup_Forward.xml");
    models.push_back(pathToModels + "/Arm26/arm26.osim");
    models.push_back(pathToModels + "/BouncingBlock/bouncing_block_weak_spring.osim");
    models.push_back(pathToModels + "/BouncingBlock/bouncing_block.osim");
    models.push_back(pathToModels + "/DoublePendulum/double_pendulum.osim");
    models.push_back(pathToModels + "/Gait10dof18musc/gait10dof18musc.osim");
//    models.push_back(pathToModels + "/Jumper/DynamicJumperModel.osim");
    models.push_back(pathToModels + "/Leg6Dof9Musc/leg6dof9musc.osim");
    models.push_back(pathToModels + "/Leg39/leg39.osim");
    models.push_back(pathToModels + "/RajagopalModel/Rajagopal2015.osim");
    models.push_back(pathToModels + "/SoccerKick/SoccerKickingModel.osim");
    models.push_back(pathToModels + "/ToyLanding/ToyLandingModel.osim");
    models.push_back(pathToModels + "/Tug_of_War/Tug_of_War.osim");
    models.push_back(pathToModels + "/WalkerModel/WalkerModel.osim");
//    models.push_back(pathToModels + "/WristModel/wrist.osim");




    std::ofstream myfile;
    myfile.open(pathToMain + "/solver_results.txt");
    for (auto& modeli : models){
        if (modeli.substr(modeli.length()-3,modeli.length()) == "xml"){
            // FORWARD TOOL
            myfile << "\n" << modeli << "\n";
            for (int i=0; i<integrators.size(); i++){
                std::chrono::milliseconds simTime{0};
                for (int ii=0; ii<nSim; ii++){
                    OpenSim::ForwardTool model{modeli};
                    model.setFinalTime(1.0);
                    model.setIntegratorMethod(i);

                    auto before = std::chrono::high_resolution_clock::now();
                    model.run();
                    auto after = std::chrono::high_resolution_clock::now();
                    auto dt = after - before;
                    simTime += std::chrono::duration_cast<std::chrono::milliseconds>(dt);
                }
                simTime /= nSim;
                myfile << "simTime("<<i<<"): " << simTime.count() << "\t"
                       << integrators[i] << "\n";
                std::cout << "simTime("<<i<<"): " << simTime.count() << "\t"
                          << integrators[i] << std::endl;
            }
        }

        else{
            // CLASSIC OSIM INTEGRATION
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
                std::chrono::milliseconds simTime{0};
                for (int ii=0; ii<nSim; ii++){
                    OpenSim::Manager manager(model);
                    manager.setIntegratorMethod(OpenSim::Manager::IntegratorMethod(i));
                    manager.initialize(state);

                    auto before = std::chrono::high_resolution_clock::now();
                    manager.integrate(2);
                    auto after = std::chrono::high_resolution_clock::now();
                    auto dt = after - before;
                    simTime += std::chrono::duration_cast<std::chrono::milliseconds>(dt);
                }
                simTime /= nSim;
                myfile << "simTime("<<i<<"): " << simTime.count() << "\t"
                       << integrators[i] << "\n";
                std::cout << "simTime("<<i<<"): " << simTime.count() << "\t"
                          << integrators[i] << std::endl;
            }
        }
    }
    myfile.close();
    return 0;
}
