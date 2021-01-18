#ifndef INTERP_H
#define INTERP_H

#include <vector>
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using std::vector;
using std::pair;
using OpenSim::Coordinate;
using OpenSim::Component;
using OpenSim::Muscle;
using OpenSim::GeometryPath;
using std::vector;
using std::pair;

struct Nonzero_conditions final {
    double input_val;
    double nonzero_ma;
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

template<typename T>
void linspace(vector<double> &linspaced, T start_in, T end_in, int num_in){
// from: https://stackoverflow.com/questions/27028226/python-linspace-in-c
  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0){
      return;
  }
  if (num == 1){
      linspaced.push_back(start);
      return;
  }

  double delta = (end - start) / (num - 1);
  for(int i=0; i < num-1; ++i){
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
}

struct Discretization{
    double begin;
    double end;
    int nPoints;
    double gridsize;
};


class interp{
    private:
        // dimension of the interpolation (e.g. two states -> 2D)
        int dimension;
        // number of points considered in constructing the polynomial
        int nInterPoints = 4;
        int g = 1;
        // order of the interpolation polynomial function
//        int polyOrder = 5;
        // vector containing the orthogonal discretization vectors
        // e.g. disc[0] = xrange, disc[1] = yrange etc.
        vector<vector<double>> discretization;
        vector<double> discSizes;
        // array containing the evaluations over the discretizations
        vector<pair<vector<int>,double>> evalsPair;
        vector<double> evals;
        // vector containing index closest to discretization point
        vector<int> n;
        // vector containing fraction of closest index to discretization point to next
        vector<double> u;
        // array of polynomial evaluation
        vector<vector<double>> beta;
        // vector of an index in the evalsPair
        vector<int> loc;

        // OPENSIM INTEGRATION
        vector<Discretization> dS;


    public:
        // getting
        vector<double> getRange(int i) {return discretization[i];}
        int getDimension() {return dimension;}

        // template member functions
        double getEval();
        double getEvalFast();
        double getInterp(vector<double> x);
        double getInterpStruct(vector<double> x);
        double getInterpDer(vector<double> x, int coordinate);

        double interpCubicHermiteSpline(vector<double> x, int derivativeOrder);
        void computeBasisFunctions(vector<vector<double>> &beta,
                                   vector<double> u, int order);
        void computeBasisFunctionsDerivatives(vector<vector<double>> &beta,
                                   vector<double> u, int order);
        double binomialCoefficient(int n, int k);


        // OLD METHOD
        explicit interp(vector<vector<double>> discretizationIn,
                        vector<pair<vector<int>,double>> evalsPair)
            : discretization(discretizationIn),
              dimension(discretizationIn.size()),
              evalsPair(evalsPair),
              n(dimension,0),
              u(dimension,0),
              loc(dimension,0)
        {
            assert(discretization.size() == evalsPair[0].first.size());

            // allow it to work with the new struct method
            Discretization dc;
            for (int i=0; i<dimension; i++){
                dc.begin = discretization[i][0];
                dc.end = discretization[i][discretization[i].size()];
                dc.nPoints = discretization[i].size();
                dc.gridsize = (dc.end - dc.begin)/dc.nPoints;
                dS.push_back(dc);
            }

            for (int i=0; i<dimension; i++){
                beta.push_back({0,0,0,0});
                discSizes.push_back(discretization[i].size());
            }
            // I'm aware this is still stupid but it is what it is for now
            if (dimension == 2){
                vector<int> test(2,0);
                for (int i=0; i<discretization[0].size(); i++){
                    for (int j=0; j<discretization[1].size(); j++){
                        test[0] = i; test[1] = j;
                        for (int k=0; k<evalsPair.size(); k++){
                            if (evalsPair[k].first == test){
                                evals.push_back(evalsPair[k].second);
                            }
                        }
                    }
                }
            } else if (dimension == 1){
                vector<int> test(1,0);
                for (int i=0; i<discretization[0].size(); i++){
                    test[0] = i;
                    for (int k=0; k<evalsPair.size(); k++){
                        if (evalsPair[k].first == test){
                            evals.push_back(evalsPair[k].second);
                        }
                    }
                }
            }
            std::cout << evals.size() << std::endl;
        }


        // 1 COORDINATE
        explicit interp(OpenSim::Muscle const& m,
                        Coordinate const& c1,
                        SimTK::State& st,
                        vector<Discretization>& discretizationData)
            : dimension(discretizationData.size()),
              n(dimension,0),
              u(dimension,0),
              loc(dimension,0),
              dS(discretizationData)
        {
            assert(discretizationData.size() == 1);

            GeometryPath const& musc_path = m.getGeometryPath();

            // just make it for using the old getInterp method
            vector<double> dc_;
            for (int i=0; i<dimension; i++){
                beta.push_back({0,0,0,0});

                dc_.clear();
                linspace(dc_,dS[i].begin,dS[i].end,dS[i].nPoints);
                discretization.push_back(dc_);
                discSizes.push_back(dS[i].nPoints);
            }

            // unlock coordinates
            bool c1_was_locked = c1.getLocked(st);
            c1.setLocked(st, false);
            auto unlock_c1 = defer_action([&] { c1.setLocked(st, c1_was_locked); });
            double c1_initial_value = c1.getValue(st);
            auto reset_c1_val = defer_action([&] { c1.setValue(st, c1_initial_value); });

            // evalute the muscle length
            Discretization dc1 = dS[0];
            for (int ic1=0; ic1<dc1.nPoints; ++ic1){
                double c1v = dc1.begin + (ic1*dc1.gridsize);
                c1.setValue(st, c1v);
                evals.push_back(musc_path.getLength(st));
            }
        }

        // 2 COORDINATES
        explicit interp(OpenSim::Muscle const& m,
                        Coordinate const& c1,
                        Coordinate const& c2,
                        SimTK::State& st,
                        vector<Discretization>& discretizationData)
            : dimension(discretizationData.size()),
              n(dimension,0),
              u(dimension,0),
              loc(dimension,0),
              dS(discretizationData)
        {
            assert(discretizationData.size() == 2);

            GeometryPath const& musc_path = m.getGeometryPath();

            // just make it for using the old getInterp method
            vector<double> dc_;
            for (int i=0; i<dimension; i++){
                beta.push_back({0,0,0,0});

                dc_.clear();
                linspace(dc_,dS[i].begin,dS[i].end,dS[i].nPoints);
                discretization.push_back(dc_);
                discSizes.push_back(dS[i].nPoints);
            }

            // unlock coordinates
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
            // evalute the muscle length

            Discretization dc1 = dS[0];
            Discretization dc2 = dS[1];
            for (int ic1=0; ic1<dc1.nPoints; ++ic1){
                double c1v = dc1.begin + (ic1*dc1.gridsize);
                c1.setValue(st, c1v);
                for (int ic2=0; ic2<dc2.nPoints; ++ic2){
                    double c2v = dc2.begin + (ic2*dc2.gridsize);
                    c2.setValue(st, c2v);
                    evals.push_back(musc_path.getLength(st));
                }
            }
        }

        // 2 COORDINATES GIVEN N DISC POINTS
        explicit interp(OpenSim::Muscle const& m,
                        Coordinate const& c1,
                        Coordinate const& c2,
                        SimTK::State& st,
                        vector<int>& discretizationNPoints)
            : dimension(discretizationNPoints.size()),
              n(dimension,0),
              u(dimension,0),
              loc(dimension,0)
        {
            assert(discretizationNPoints.size() == 2);

            GeometryPath const& musc_path = m.getGeometryPath();

            // unlock coordinates
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

            // make discretization objects for interpolation class instance
            Discretization dc1, dc2;
            dc1.begin = c1.getRangeMin();
            dc1.end = c1.getRangeMax();
            dc1.nPoints = discretizationNPoints[0];
            dc1.gridsize = (dc1.end-dc1.begin) / dc1.nPoints;

            dc2.begin = c2.getRangeMin();
            dc2.end = c2.getRangeMax();
            dc2.nPoints = discretizationNPoints[1];
            dc2.gridsize = (dc2.end-dc2.begin) / dc2.nPoints;

            dS.push_back(dc1);
            dS.push_back(dc2);

            // just make it for using the old getInterp method
            vector<double> dc_;
            for (int i=0; i<dimension; i++){
                beta.push_back({0,0,0,0});

                dc_.clear();
                linspace(dc_,dS[i].begin,dS[i].end,dS[i].nPoints);
                discretization.push_back(dc_);
                discSizes.push_back(dS[i].nPoints);
            }

            // evalute the muscle length
            for (int ic1=0; ic1<dc1.nPoints; ++ic1){
                double c1v = dc1.begin + (ic1*dc1.gridsize);
                c1.setValue(st, c1v);
                for (int ic2=0; ic2<dc2.nPoints; ++ic2){
                    double c2v = dc2.begin + (ic2*dc2.gridsize);
                    c2.setValue(st, c2v);
                    evals.push_back(musc_path.getLength(st));
                }
            }
        }
};

#endif // INTERP_H














//// N COORDINATES
//explicit interp(OpenSim::Muscle const& m,
//                Coordinate const** cBegin,
//                Coordinate const** cEnd,
//                SimTK::State& st,
//                vector<Discretization>& discretizationData)
//    : dimension(discretizationData.size()),
//      n(dimension,0),
//      u(dimension,0),
//      loc(dimension,0),
//      dS(discretizationData)
//{
//    std::ptrdiff_t n = end-begin;
//    assert(discretizationData.size() == (int)n);

//    GeometryPath const& musc_path = m.getGeometryPath();

//    // just make it for using the old getInterp method
//    vector<double> dc_;
//    for (int i=0; i<dimension; i++){
//        beta.push_back({0,0,0,0});

//        dc_.clear();
//        linspace(dc_,dS[i].begin,dS[i].end,dS[i].nPoints);
//        discretization.push_back(dc_);
//        discSizes.push_back(dS[i].nPoints);
//    }

//    // unlock coordinates
//    for (int i=0; i<dimension; i++){

//        bool c_was_locked = c.getLocked(st);
//        c.setLocked(st, false);
//        auto unlock_c = defer_action([&] { c.setLocked(st, c_was_locked); });
//        double c_initial_value = c.getValue(st);
//        auto reset_c_val = defer_action([&] { c.setValue(st, c_initial_value); });
//    }

//    // evalute the muscle length
//    Discretization dc1 = dS[0];
//    Discretization dc2 = dS[1];
//    for (int ic1=0; ic1<dc1.nPoints; ++ic1){
//        double c1v = dc1.begin + (ic1*dc1.gridsize);
//        c1.setValue(st, c1v);
//        for (int ic2=0; ic2<dc2.nPoints; ++ic2){
//            double c2v = dc2.begin + (ic2*dc2.gridsize);
//            c2.setValue(st, c2v);
//            evals.push_back(musc_path.getLength(st));
//        }
//    }
//}
