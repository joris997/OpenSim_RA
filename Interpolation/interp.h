#include <vector>
#ifndef INTERP_H
#define INTERP_H

class interp{
    private:
        // dimension of the interpolation (e.g. two states -> 2D)
        int dimension;
        // number of points considered in constructing the polynomial
        int nInterPoints=4;
        // order of the interpolation polynomial function
        int polyOrder = 5;
        // vector containing the orthogonal discretization vectors
        // e.g. disc[0] = xrange, disc[1] = yrange etc.
        std::vector<std::vector<double>> discretization;
        // array containing the evaluations over the discretizations
        double evals[10][10];
        std::vector<std::pair<std::vector<int>,double>> evalsPair;


    public:
        // getting
        std::vector<double> getRange(int i) {return discretization[i];}
        int getDimension() {return dimension;}
        int getNInterPoints() {return nInterPoints;}
        int getPolyOrder() {return polyOrder;}

        // setting
        void setNInterPoints(int newNInterPoints) {nInterPoints = newNInterPoints;}
        void setPolyOrder(int newPolyOrder) {polyOrder = newPolyOrder;}

        // template member functions
        double getEval(std::vector<int> loc);
        double interpGridSpline(std::vector<double> x);
        double interpCubicHermiteSpline(std::vector<double> x, int derivativeOrder);
        void computeBasisFunctions(std::vector<std::vector<double>> &beta,
                                   std::vector<double> u, int order);
        void computeBasisFunctionsDerivatives(std::vector<std::vector<double>> &beta,
                                   std::vector<double> u, int order);
        double binomialCoefficient(int n, int k);


//        explicit interp(std::vector<std::vector<double>> discretization)
//            : discretization(discretization),
//              dimension(discretization.size())
//        {
//            double discrLengths[discretization.size()];
//            for (int i=0; i<discretization.size(); i++){
//                discrLengths[i] = -discretization[i].size();
//            }
//            while (discrLengths[0] != 0){

//                for (size_t x = discretization.size(); x--;){
//                    if (discrLengths[x-1] != 0){
//                        discrLengths[x-1] += 1;
//                        break;
//                    }
//                }
//            }
//        }
        explicit interp(std::vector<std::vector<double>> discretization,
                        std::vector<std::pair<std::vector<int>,double>> evals)
            : discretization(discretization),
              dimension(discretization.size()),
              evalsPair(evals)
        {}


//        explicit interp(std::vector<double> xRange, std::vector<double> yRange)
//            : dimension(2)
//        {
//            for (int i=0; i<xRange.size(); i++){
//                for (int ii=0; ii<yRange.size(); ii++){
//                    evals[i][ii] = xRange[i]*yRange[ii];
//                }
//            }
//            discretization.push_back(xRange);
//            discretization.push_back(yRange);
//        }

//        explicit interp(std::vector<double> xRange, std::vector<double> yRange, std::vector<double> zRange)
//            : dimension(3)
//        {
//            for (int i=0; i<xRange.size(); i++){
//                for (int ii=0; ii<yRange.size(); ii++){
//                    evals[i][ii] = xRange[i]*yRange[ii];
//                }
//            }
//            discretization.push_back(xRange);
//            discretization.push_back(yRange);
//            discretization.push_back(zRange);
//        }
};

#endif // INTERP_H
