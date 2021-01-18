#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

#include <iostream>
#include <cstring>
#include <cmath>
#include "interp.h"


using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::GeometryPath;


#include <vector>

template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in){
  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) {
      linspaced.push_back(start);
      return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i){
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}


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

    if (*end != '\0' or v >= HUGE_VAL or v <= -HUGE_VAL) {
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

// defer an action until the destruction of this wrapper
template<typename Callback>
struct Defer final {
    Callback cb;
    Defer(Callback _cb) : cb{std::move(_cb)} {
    }
    Defer(Defer const&) = delete;
    Defer(Defer&&) noexcept = default;
    Defer& operator=(Defer const&) = delete;
    Defer& operator=(Defer&&) = delete;
    ~Defer() noexcept {
        cb();
    }
};

// factory function to Defer
template<typename Callback>
Defer<Callback> defer_action(Callback cb) {
    return Defer<Callback>{std::move(cb)};
}

struct Nonzero_conditions final {
    double input_val;
    double nonzero_ma;
};

// returns `true` and populates `out` if OpenSim::Coordinate `c` affects
// OpenSim::Muscle `m` when it changes within SimTK::State `s`
static bool coord_affects_muscle(
    OpenSim::Muscle const& m,
    Coordinate const& c,
    SimTK::State& state,
    Nonzero_conditions& out) {

    bool prev_locked = c.getLocked(state);
    auto reset_locked = defer_action([&] { c.setLocked(state, prev_locked); });
    double prev_val = c.getValue(state);
    auto reset_val = defer_action([&] { c.setValue(state, prev_val); });

    c.setLocked(state, false);

    static constexpr int num_steps = 3;
    double start = c.getRangeMin();
    double end = c.getRangeMax();
    double step = (end - start) / num_steps;

    for (double v = start; v <= end; v += step) {
        c.setValue(state, v);
        double ma = m.getGeometryPath().computeMomentArm(state, c);
        if (std::abs(ma) > 0.001) {
            out.input_val = v;
            out.nonzero_ma = ma;
            return true;
        }
    }
    return false;
}

static void perform_1d_spline_fit(
        OpenSim::Muscle const& m,
        OpenSim::Coordinate const&,
        SimTK::State const&) {
    std::cerr << m.getName() << ": 1D fitting TODO" << std::endl;
}

static void perform_2d_spline_fit(
        OpenSim::Muscle const& m,
        OpenSim::Coordinate const& c1,
        OpenSim::Coordinate const& c2,
        SimTK::State& st) {

    std::ofstream out;
    {
        std::stringstream outfile_name;
        outfile_name << "/tmp/2d_";
        outfile_name << m.getName() << "_";
        outfile_name << c1.getName() << "_";
        outfile_name << c2.getName() << ".txt";

        out.open(outfile_name.str());

        if (!out) {
            throw std::runtime_error{outfile_name.str() + ": error opening file"};
        }

        std::cerr << "writing fit to: " << outfile_name.str() << std::endl;
    }

    std::string musc_name = m.getName();
    GeometryPath const& musc_path = m.getGeometryPath();

    // header
    out << c1.getName() << " " << c2.getName() << std::endl;

    // coordinates need to be unlocked to do this
    bool c1_was_locked = c1.getLocked(st);
    c1.setLocked(st, false);
    auto unlock_c1 = defer_action([&] { c1.setLocked(st, c1_was_locked); });
    double c1_initial_value = c1.getValue(st);
    auto reset_c1_val = defer_action([&] { c1.setValue(st, c1_initial_value); });

    bool c2_was_locked = c2.getLocked(st);
    c2.setLocked(st, false);
    auto unlock_c2 = defer_action([&] { c2.setLocked(st, c2_was_locked); });
    double c2_initial_value = c2.getValue(st);
    auto reset_c2_val = defer_action([&] { c2.setValue(st, c2_initial_value); });

    // iterate through the coordinates to map out a 2D surface of points which
    // can be plotted externally (e.g. by matplotlib)
    static constexpr int num_steps = 50;

    double c1_start = c1.getRangeMin();
    double c1_end = c1.getRangeMax();
    double c1_step = (c1_end - c1_start) / num_steps;

    double c2_start = c2.getRangeMin();
    double c2_end = c2.getRangeMax();
    double c2_step = (c2_end - c2_start) / num_steps;

    std::vector<std::vector<double>> discretization;
    discretization.push_back(linspace(c1_start, c1_end, num_steps));
    discretization.push_back(linspace(c2_start, c2_end, num_steps));
//    std::cerr << "discretization@c1: (" << discretization[0].size() << ")";
    for (double c1v : discretization[0]) {
//        std::cerr << c1v << ", ";
    }
//    std::cerr << std::endl;
//    std::cerr << "discretization@c2: (" << discretization[1].size() << ")";
    for (double c2v : discretization[1]) {
//        std::cerr << c2v << ", ";
    }
//    std::cerr << std::endl;

    std::vector<std::pair<std::vector<int>, double>> evals;

    for (int c1s = 0; c1s < num_steps; ++c1s) {
        double c1v = c1_start + (c1s * c1_step);
        c1.setValue(st, c1v);
        for (int c2s = 0; c2s < num_steps; ++c2s) {
            double c2v = c2_start + (c2s * c2_step);
            c2.setValue(st, c2v);
            double len = musc_path.getLength(st);

            evals.push_back({{c1s, c2s}, len});

            // c1 c2 result
            out << c1v << " " << c2v << " " << len << std::endl;
        }
    }
    c1.setValue(st, c1_initial_value);
    c2.setValue(st, c2_initial_value);


    for (auto const& p : evals) {
        for (int cv : p.first) {
//            std::cerr << cv << ",";
        }
//        std::cerr << ": " << p.second << std::endl;
    }

    if (discretization[0].size() * discretization[1].size() != evals.size()) {
        throw std::runtime_error{"incorrect number of discretizations"};
    }

    interp interpolator{std::move(discretization), std::move(evals)};

    std::ofstream real_v_interp{"/tmp/real_vs_interp"};
    assert(real_v_interp.good());
    real_v_interp << c1.getName() << "\t" << c2.getName() << "\treal_len\tinterp_len" << std::endl;

    // eval comparison
    {
        int num_steps = 100;

        double c1_start = c1.getRangeMin();
        double c1_end = c1.getRangeMax();
        double c1_step = (c1_end - c1_start) / num_steps;

        double c2_start = c2.getRangeMin();
        double c2_end = c2.getRangeMax();
        double c2_step = (c2_end - c2_start) / num_steps;

        std::chrono::microseconds real_us{0};
        std::chrono::microseconds interp_us{0};
        int count = 0;
        for (int c1s = 0; c1s < (num_steps-7); ++c1s) {
            double c1v = (c1_start + (0.5*c1_step)) + (c1s * c1_step);
            c1.setValue(st, c1v);
            for (int c2s = 0; c2s < (num_steps-7); ++c2s) {
                double c2v = (c2_start + (0.5*c2_step)) + (c2s * c2_step);
                c2.setValue(st, c2v);

                auto real_before = std::chrono::high_resolution_clock::now();
                double real_len = musc_path.getLength(st);
                auto real_after = std::chrono::high_resolution_clock::now();
                auto real_dt = real_after - real_before;
                real_us += std::chrono::duration_cast<std::chrono::microseconds>(real_dt);

                auto interp_before = std::chrono::high_resolution_clock::now();
                double interp_len = interpolator.getInterp({c1v, c2v});
                auto interp_after = std::chrono::high_resolution_clock::now();
                auto interp_dt = interp_after - interp_before;
                interp_us += std::chrono::duration_cast<std::chrono::microseconds>(interp_dt);

                ++count;
//                std::cerr<< "r: " << real_len << " i: " << interp_len << std::endl;
                real_v_interp << c1v << "\t" << c2v << "\t" << real_len << "\t" << interp_len << std::endl;
            }
//            std::cerr << "step" << std::endl;
        }
        real_us /= count;
        interp_us /= count;
        real_v_interp << "real_us: " << std::chrono::duration<double>(real_us).count() << "\tinterp_us: " << std::chrono::duration<double>(interp_us).count() << std::endl;

        std::cerr << "muscle: " << m.getName() << " c1: " << c1.getName() << " c2: " << c2.getName() << std::endl;
        std::cerr << "real micros = " << real_us.count() << std::endl;
        std::cerr << "interp micros = " << interp_us.count() << std::endl;
        std::cin.ignore();
    }

    c1.setValue(st, c1_initial_value);
    c2.setValue(st, c2_initial_value);


//    throw std::runtime_error{"done: " + m.getName()};
}

static void perform_multidimensional_spline_fit(
        OpenSim::Muscle const& m,
        Coordinate const** begin,
        Coordinate const** end,
        SimTK::State const&) {
    std::ptrdiff_t n = end - begin;
    std::cerr << m.getName() << ": cannot fit n-dimensional spline yet (coords = " << n << ")" << std::endl;
}

static void perform_spline_analysis(OpenSim::Model& model) {
    // delimited file of:
    //
    // $muscle_name $coordinate_name $nonzero_ma
    std::ofstream outfile{"/tmp/coords"};
    if (!outfile.good()) {
        throw std::runtime_error{"error opening outfile"};
    }

    // delimited file of:
    //
    // $muscle_name($num_nonzero_ma_coords): $nonzero_coordinate_names*
    std::ofstream assocs{"/tmp/assocs"};
    if (!assocs.good()) {
        throw std::runtime_error{"error opening assocs file"};
    }

    SimTK::State& state = model.initSystem();
    model.equilibrateMuscles(state);
    model.realizeVelocity(state);

    std::vector<Muscle const*> muscles;
    int max_muscle_namelen = 0;
    for (Muscle const& m : model.getComponentList<Muscle>()) {
//        if (m.getName() == "soleus_r") {
            max_muscle_namelen = std::max(max_muscle_namelen, static_cast<int>(m.getName().size()));
            muscles.push_back(&m);
//        }
    }

    std::vector<Coordinate const*> coords;
    int max_coord_namelen = 0;
    for (Coordinate const& c : model.getComponentList<Coordinate>()) {
        if (c.getMotionType() != Coordinate::MotionType::Coupled) {
            max_coord_namelen = std::max(max_coord_namelen, static_cast<int>(c.getName().size()));
            coords.push_back(&c);
        }
    }

    // sort data by $muscle_name (primary) then $coordinate_name (secondary)
    // for easier eyeballing

    std::sort(muscles.begin(), muscles.end(), [](Muscle const* m1, Muscle const* m2) {
        return m1->getName() < m2->getName();
    });

    std::sort(coords.begin(), coords.end(), [](Coordinate const* c1, Coordinate const* c2) {
        return c1->getName() < c2->getName();
    });

    for (Muscle const* m : muscles) {
        std::vector<Coordinate const*> affecting_coords;

        Nonzero_conditions cond;
        for (Coordinate const* c : coords) {

            switch (c->getMotionType()) {
            case Coordinate::MotionType::Rotational:
                outfile << "rotational    ";
                break;
            case Coordinate::MotionType::Translational:
                outfile << "translational ";
                break;
            case Coordinate::MotionType::Coupled:
                outfile << "coupled       ";
                break;
            default:
                outfile << "undefined     ";
                break;
            }

            outfile << std::setw(max_muscle_namelen + 1) << std::left << m->getName();
            outfile << std::setw(max_coord_namelen + 1) << std::left << c->getName();

            if (coord_affects_muscle(*m, *c, state, cond)) {
                affecting_coords.push_back(c);
                outfile << cond.nonzero_ma;
            } else {
                outfile << 0.0;
            }
            outfile << std::endl;
        }

        if (!affecting_coords.empty()) {
            assocs << m->getName() << "(" << affecting_coords.size() << "): ";
            for (size_t i = 0; i < affecting_coords.size()-1; ++i) {
                assocs << affecting_coords[i]->getName() << " ";
            }
            assocs << affecting_coords[affecting_coords.size()-1]->getName() << std::endl;
        }

        // TODO: if the muscle is affected by 1/2 coordinates then we can try
        // to perform a spline fit
        switch (affecting_coords.size()) {
        case 0:
            std::cerr << m->getName() << ": not affected by any coords: skipping" << std::endl;
            break;
        case 1:
            perform_1d_spline_fit(*m, *affecting_coords[0], state);
            break;
        case 2:
            perform_2d_spline_fit(*m, *affecting_coords[0], *affecting_coords[1], state);
            break;
        default:
            perform_multidimensional_spline_fit(*m, affecting_coords.data(), affecting_coords.data() + affecting_coords.size(), state);
            break;
        }
    }
}

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
