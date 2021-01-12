#include <vector>
#include <iostream>
#ifndef INTERP_H
#define INTERP_H

using std::vector;
using std::pair;

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



    public:
        // getting
        vector<double> getRange(int i) {return discretization[i];}
        int getDimension() {return dimension;}
//        int getNInterPoints() {return nInterPoints;}
//        int getPolyOrder() {return polyOrder;}

        // setting
//        void setNInterPoints(int newNInterPoints) {nInterPoints = newNInterPoints;}
//        void setPolyOrder(int newPolyOrder) {polyOrder = newPolyOrder;}

        // template member functions
        double getEval();
        double getEvalFast();
        double getInterp(vector<double> x);
        double getInterpDer(vector<double> x, int coordinate);

        double interpCubicHermiteSpline(vector<double> x, int derivativeOrder);
        void computeBasisFunctions(vector<vector<double>> &beta,
                                   vector<double> u, int order);
        void computeBasisFunctionsDerivatives(vector<vector<double>> &beta,
                                   vector<double> u, int order);
        double binomialCoefficient(int n, int k);


        explicit interp(vector<vector<double>> discretization,
                        vector<pair<vector<int>,double>> evalsPair)
            : discretization(discretization),
              dimension(discretization.size()),
              evalsPair(evalsPair),
              n(dimension,0),
              u(dimension,0),
              loc(dimension,0)
        {
            for (int i=0; i<dimension; i++){
                beta.push_back({0,0,0,0});
                discSizes.push_back(discretization[i].size());
            }
            // I'm aware this is still stupid but it is what it is for now
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
            std::cout << evals.size() << std::endl;
        }
};

#endif // INTERP_H
