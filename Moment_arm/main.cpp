#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

#include <iostream>
#include <cstring>
#include <cmath>


using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::Array;

static const char HELP[] =
R"(usage: fd [--help][--no-visualizer][--no-wrapping][--format=<format>]
             model final_time
)";

// skip `prefix` in `s`
//
// if a prefix was skipped (i.e. `s` begins with `prefix`):
//     - returns `true`
//     - sets `out` to the location in `s` where `prefix` ends
// else:
//     - returns `false`
static bool skip_prefix(char const* s, char const* prefix, char const** out) {
    do {
        if (*prefix == '\0') {
            *out = s;
            return true;
        }
    } while (*s++ == *prefix++);
    return false;
}

// parse `s`, a string representation of a double
//
// if parsing was successful:
//     - returns `true`
//     - sets `out` to the parsed value
// else:
//     - returns false
static bool safe_parse_double(char const* s, double* out) {
    char* end;
    double v = strtod(s, &end);

    if (*end != '\0' or v == HUGE_VAL or v == -HUGE_VAL) {
        return false;  // not a number
    }

    *out = v;
    return true;
}

// basic OpenSim::Analysis that prints info about the simulation as it runs
struct Log_emitter final : public OpenSim::Analysis {
    OpenSim::Model const& model;
    char const* format;

    Log_emitter(OpenSim::Model const& _model, char const* _format) :
        model{_model},
        format{_format} {

        if (!_format) {
            throw std::runtime_error{"nullptr format passed to Log_emitter"};
        }
        std::cerr << "AKINFO: time,prescribeQcalls" << std::endl;
    }

    int step(const SimTK::State& s, int) override {
        std::cerr << "AKINFO: "
                  << s.getTime() << ','
                  << model.getMultibodySystem().getNumPrescribeQCalls() << std::endl;
        return 0;
    }

    Log_emitter* clone() const override {
        return new Log_emitter{model, format};
    }

    const std::string& getConcreteClassName() const override {
        static std::string name = "Log_emitter";
        return name;
    }
};

static void perform_spline_analysis(OpenSim::Model& model) {
    // HACK: this just always outputs moment arms here
    std::ofstream outfile{"tmp/coords"};
    if (!outfile.good()) {
        throw std::runtime_error{"error opening outfile"};
    }
    SimTK::State& state = model.initSystem();
//    std::cout << "state: " << state.toString() << std::endl;
    model.equilibrateMuscles(state);
    model.realizeVelocity(state);

    // collect list of coordinates to iterate over up-front, so that we can
    // perform any sorting and column sizing ahead of time
    std::vector<OpenSim::Coordinate const*> coords;
    {
        for (auto const& c : model.getComponentList<OpenSim::Coordinate>()) {
            coords.push_back(&c);
        }
    }

    // get longest coordinate name, so that we can print a column-aligned format
    // (easier to read by eye)
    size_t max_coord_namelen = 0;
    for (auto const* coord : coords) {
        max_coord_namelen = std::max(max_coord_namelen, coord->getName().size());
    }

    // same for muscle
    size_t max_musc_namelen = 0;
    for (auto const& m : model.getComponentList<OpenSim::Muscle>()) {
        max_musc_namelen = std::max(max_musc_namelen, m.getName().size());
    }

    // callback that is called once per muscle-coordinate pair
    //
    // THIS IS WHERE THE ANALYSIS MOSTLY HAPPENS
    auto handle_coord = [&](Muscle const& m, Coordinate const& c, char const* label) {
//        // compute path's moment arms for all gen. coord.
//        // added by Joris as a double check
//        if (m.getName() == "gaslat_l"){
//            Array<double> momentArms;
//            m.getGeometryPath().computeMomentArms(state,momentArms);
//        }

        bool prev_locked = c.getLocked(state);
        double prev_val = c.getValue(state);

        c.setLocked(state, false);

        static constexpr int num_steps = 20;
        double nonzero_v = 0.0;
        double start = c.getRangeMin();
        double end = c.getRangeMax();
        double step = (end - start) / num_steps;
        for (double v = start; v <= end; v += step) {
            c.setValue(state, v);
            double ma = m.getGeometryPath().computeMomentArm(state, c);
            if (std::abs(ma) > SimTK::Eps) {
                nonzero_v = ma;
                break;
            }
        }

        outfile << label;
        outfile << std::setw(max_musc_namelen + 1) << std::left << m.getName();
        outfile << std::setw(max_coord_namelen + 1) << std::left << c.getName();
        if (nonzero_v != 0.0) {
            outfile << nonzero_v;
        } else {
            outfile << "NONE";
        }
        outfile << std::endl;

        c.setValue(state, prev_val);
        c.setLocked(state, prev_locked);

        // as in, the coord is nonzero *somewhere*
        return nonzero_v != 0.0;
    };

    // output #2: list of muscle-to-coordinates associations
    std::vector<std::string> associated_coords;
    std::ofstream assocs{"tmp/assocs"};

    for (auto const& m : model.getComponentList<OpenSim::Muscle>()) {
        if (m.getName() == "gaslat_l"){
        associated_coords.clear();

        for (OpenSim::Coordinate const* c : coords) {
            std::cout << "coord: " << c->getName() << std::endl;
            char const* label = "undefined     ";

            switch (c->getMotionType()) {
            case Coordinate::MotionType::Rotational:
                label = "rotational    ";
                break;
            case Coordinate::MotionType::Translational:
                label = "translational ";
                break;
            case Coordinate::MotionType::Coupled:
                label = "coupled       ";
                break;
            default:
                break;
            }

            if (handle_coord(m, *c, label)) {
                associated_coords.push_back(c->getName());
            }
        }

        if (!associated_coords.empty()) {
            assocs << m.getName() << "(" << associated_coords.size() << "): ";
            for (size_t i = 0; i < associated_coords.size()-1; ++i) {
                assocs << associated_coords[i] << " ";
            }
            assocs << associated_coords[associated_coords.size()-1] << std::endl;
        }
        }
    }
}

// SHOULD NOT BE INTERACTING: pelvis_list, gaslat_l
// checks: implementation in analysis 'handle_coord'
int main(int argc, char** argv) {
    // skip app name
    --argc;
    ++argv;

    bool visualize = true;
    bool disable_wrapping = false;
    char const* format = nullptr;

    // handle named args
    while (argc > 0) {
        char const* arg = argv[0];

        if (arg[0] != '-') {
            break;
        }

        if (!strcmp(arg, "--help")) {
            std::cout << HELP;
            return 0;
        } else if (!strcmp(arg, "--no-visualizer")) {
            visualize = false;
        } else if (!strcmp(arg, "--no-wrapping")) {
            disable_wrapping = true;
        } else if (skip_prefix(arg, "--format", &arg)) {
            // TODO: the format arg doesn't actually use the format string at
            //       the moment - it just prints whatever Log_emitter emits
            if (*arg == '=') {
                ++arg;
                format = arg;
            } else if (!*arg) {
                if (argc < 2) {
                    std::cerr << "fd: no format string given for --format" << std::endl;
                    std::cerr << HELP;
                    return -1;
                }
                format = argv[1];
                --argc;
                ++argv;
            }
        } else {
            std::cerr << "fd: unknown option: " << arg << std::endl;
            std::cerr << HELP;
            return -1;
        }

        --argc;
        ++argv;
    }

    // handle unnamed args
    switch (argc) {
    case 0:
        std::cerr << "fd: missing arguments: model final_time" << std::endl;
        std::cerr << HELP;
        return -1;
    case 1:
        std::cerr << "fd: missing final_time argument" << std::endl;
        std::cerr << HELP;
        return -1;
    case 2:
        break;
    default:
        std::cerr << "fd: invalid number of arguments" << std::endl;
        std::cerr << HELP;
        return -1;
    }

    double final_time;
    if (not safe_parse_double(argv[1], &final_time)) {
        std::cerr << "fd: " << argv[1] << ": invalid final time (not a number)" << std::endl;
        return -1;
    }

    // load user-provided osim file
    OpenSim::Model model{argv[0]};

    if (disable_wrapping) {
        OpenSim::ComponentList<OpenSim::WrapObjectSet> l =
                model.updComponentList<OpenSim::WrapObjectSet>();
        for (OpenSim::WrapObjectSet& wos : l) {
            for (int i = 0; i < wos.getSize(); ++i) {
                OpenSim::WrapObject& wo = wos[i];
                wo.set_active(false);
            }
        }
    }

    if (visualize) {
        model.setUseVisualizer(true);
    }



    // HACK: perform spline fitting analysis: currently just emits info
    perform_spline_analysis(model);



    SimTK::State& state = model.initSystem();
    model.equilibrateMuscles(state);

    if (format) {
        model.addAnalysis(new Log_emitter{model, format});
    }

    if (visualize) {
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
    }


    OpenSim::simulate(model, state, final_time);

    return 0;
}







//ImGui::Text("wrapping surfaces: ");
//ImGui::SameLine();
//if (ImGui::Button("disable")) {
//OpenSim::Model& m = model;
//for (OpenSim::WrapObjectSet& wos : m.updComponentList<OpenSim::WrapObjectSet>()) {
//for (int i = 0; i < wos.getSize(); ++i) {
//OpenSim::WrapObject& wo = wos[i];
//wo.set_active(false);
//wo.upd_Appearance().set_visible(false);
//}
//}
//on_user_edited_model();
//}
