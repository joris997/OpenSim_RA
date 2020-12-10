#include "interp.h"
#include <math.h>
#include <iostream>
#include <algorithm>

double interp::interpGridSpline(std::vector<double> x){
    if (x.size() != dimension){
        std::cout << "Interpolation state not equal to object's dimension" << std::endl;
        exit(EXIT_FAILURE);
    }
    // This is the main interpolation function
    // IN:  x, a vector of points within the considered interpolation range
    // OUT: eval, the interpolated value

    // get the index of the closest range value to the discretization point
    std::vector<int> n;
    for (int i=0; i<dimension; i++){
        for (int ii=1; ii<discretization[i].size(); ii++){
            if (x[i] - discretization[i][ii] < 0){
                n.push_back(ii-1);
                break;
            }
        }
    }

    // compute remaining fraction
    std::vector<double> u;
    for (int i=0; i<dimension; i++){
        u.push_back((double)(x[i]-discretization[i][n[i]])/
                (discretization[i][2]-discretization[i][1]));
    }

    int g = (nInterPoints-2)/2;

    // compute the polynomials (already evaluated)
    double beta[dimension][4];
    bool derivative = false;
    if (!derivative){
        for (int i=0; i<dimension; i++){
            // compute binomial coefficient
            beta[i][0] = (0.5*pow(u[i] - 1,3)*u[i]*(2*u[i] + 1));
            beta[i][1] = (-0.5*(u[i] - 1)*(6*pow(u[i],4) - 9*pow(u[i],3) + 2*u[i] + 2));
            beta[i][2] = (0.5*u[i]*(6*pow(u[i],4) - 15*pow(u[i],3) + 9*pow(u[i],2) + u[i] + 1));
            beta[i][3] = (-0.5*(u[i] - 1)*pow(u[i],3)*(2*u[i] - 3));
        }
    } else {
        for (int i=0; i<dimension; i++){
            // compute binomial coefficient derivatives
            beta[i][0] = 0.5*pow(u[i]-1,2)*(10*pow(u[i],2)-1);
            beta[i][1] = -0.5*u[i]*(30*pow(u[i],3) - 60*pow(u[i],2) + 27*u[i] + 4);
            beta[i][2] = 0.5*(-30*pow(u[i],4) + 60*pow(u[i],3) - 27*pow(u[i],2) + 2*u[i] - 1);
            beta[i][3] = -0.5*pow(u[i],2)*(10*pow(u[i],2) - 20*u[i] + 9);
        }
    }

    // loop over all the considered points (n-dimensional) and multiply the
    // evaluation with the weight

    // create an array containing the  lengths of each discretization direction
    int discrLoopCnt[dimension];
    for (int i=0; i<dimension; i++){
        discrLoopCnt[i] = -1;
    }

    double z;
    bool breakWhile = false;
    while (discrLoopCnt[0] < 3){
        double Beta = 1;
        for (int i=0; i<dimension; i++){
            Beta = Beta*beta[i][discrLoopCnt[i]+g];
        }

        std::vector<int> loc;
        for (int i=0; i<dimension; i++){
            loc.push_back(discrLoopCnt[i] + n[i]);
        }
        z += getEval(loc)*Beta;

        // from the back to the front, check if we're already at the maximum iteration
        // on that 'nested' for loop or else increment with 1. In short, everything
        // starts with [-1,-1,-1,...] and we keep adding ones until the array of the
        // loops becomes [ 2, 2, 2, ...]
        for (int x=dimension-1; x>=0; x--){
            if (discrLoopCnt[x] != 2){
                discrLoopCnt[x] += 1;
                break;
            }
            if (discrLoopCnt[x] == 2){
                for (int y=x; y<dimension; y++){
                    discrLoopCnt[y] = -1;
                }
            }
        }

        // CHECKING EXIT CONDITIONS
        if (breakWhile){
            break;
        }
        // loop through to check whether all are at max cnt or not
        bool allTrue = true;
        for (int i=0; i<dimension; i++){
            if (discrLoopCnt[i] != 2){
                allTrue = false;
            }
        }
        // if all are true (all are 2) set breakWhile to break on the next iteration
        if (allTrue){
            breakWhile = true;
        }
    }
    return z;
}




double interp::interpCubicHermiteSpline(std::vector<double> x, int derivativeOrder){
    // This is the main interpolation function
    // IN:  x, a vector of points within the considered interpolation range
    // OUT: eval, the interpolated value
    std::vector<int> n, pk, pkp1, pkm1, pkp2;
    std::vector<double> u, d;
    std::vector<std::vector<double>> beta, m;
    double Beta0=1, Beta1=1, Beta2=1, Beta3=1, z=0;
    int order = 3;

    // check size of input argument
    if (x.size() != dimension){
        std::cout << "Interpolation state not equal to object's dimension" << std::endl;
        exit(EXIT_FAILURE);
    }

    // get the index of the closest range value to the discretization point
    for (int i=0; i<dimension; i++){
        for (int ii=1; ii<discretization[i].size(); ii++){
            if (x[i] - discretization[i][ii] < 0){
                n.push_back(ii-1);
                break;
            }
        }
    }
//    for (int i=0; i<n.size(); i++){
//        std::cout << "n[" << i << "]: " << n[i] << std::endl;
//    }

    // compute remaining fraction
    for (int i=0; i<dimension; i++){
        d.push_back((double)(discretization[i][2]-discretization[i][1]));
    }
    for (int i=0; i<dimension; i++){
        u.push_back((double)((x[i]-discretization[i][n[i]])/(d[i])));
    }

    // compute the polynomials (already evaluated)
    if (derivativeOrder == 0){
        computeBasisFunctions(beta,u,order);
    } else if (derivativeOrder == 1){
        computeBasisFunctionsDerivatives(beta,u,order);
    } else {
        std::cout << "Order of the derivative for interpolation: " << derivativeOrder <<
                     " not supported" << std::endl;
        exit(EXIT_FAILURE);
    }
//    for (int i=0; i<dimension; i++){
//        std::cout << "sum(beta[" << i << "]): " << beta[i][0] + beta[i][1] + beta[i][2] +
//                beta[i][3] << std::endl;
//    }
//    for (int i=0; i<4; i++){
//        std::cout << "beta[0][" << i << "]: " << beta[0][i] << std::endl;
//    }

    // obtain derivative approximation m
    for (int i=0; i<dimension; i++){
        pk = n;
        pkp1 = n; pkp1[i] += 1;
        pkm1 = n; pkm1[i] -= 1;
        std::vector<double> mArray;
        mArray.push_back(0.5*((getEval(pkp1) - getEval(pk))/
                       (discretization[i][n[i]+1] - discretization[i][n[i]]) +
                       (getEval(pk) - getEval(pkm1))/
                       (discretization[i][n[i]] - discretization[i][n[i]-1])));

        pkp2 = pkp1; pkp2[i] += 1;
        mArray.push_back(0.5*((getEval(pkp2) - getEval(pkp1))/
                       (discretization[i][n[i]+2] - discretization[i][n[i]+1]) +
                       (getEval(pkp1) - getEval(n))/
                       (discretization[i][n[i]+1] - discretization[i][n[i]])));

        m.push_back(mArray);
    }

    for (int i=0; i<dimension; i++){
        Beta0 *= beta[i][0];
        Beta1 *= beta[i][1];
        Beta2 *= beta[i][2];
        Beta3 *= beta[i][3];
    }
//    std::cout << "Beta0: " << Beta0 << std::endl;
//    std::cout << "Beta1: " << Beta1 << std::endl;
//    std::cout << "Beta2: " << Beta2 << std::endl;
//    std::cout << "Beta3: " << Beta3 << std::endl;

    for (int i=0; i<dimension; i++){
        pkp1 = n; pkp1[i] += 1;
        z += (Beta0*getEval(n) + Beta1*m[i][0] + Beta2*getEval(pkp1) + Beta3*m[i][1]);
//        z += (beta[i][0]*getEval(n) + beta[i][1]*m[i][0] +
//              beta[i][2]*getEval(pkp1) + beta[i][3]*m[i][1]);
//        std::cout << "\n" << "z[" << i << "]: " <<
//                     (Beta0*getEval(n) + Beta1*m[i][0] + Beta2*getEval(pkp1) + Beta3*m[i][1]) << std::endl;
    }
    return z;
}



void interp::computeBasisFunctions(std::vector<std::vector<double>> &beta,
                                   std::vector<double> u,
                                   int order){
    // create the basis spline functions
    for (int i=0; i<dimension; i++){
        double Bk[4];
        for (int k=0; k<4; k++){
            Bk[k] = binomialCoefficient(order,k)*pow(u[i],k)*pow(1-u[i],order-k);
        }
        std::vector<double> betaArray;
        betaArray.push_back(Bk[0] + Bk[1]);
        betaArray.push_back(Bk[1]/3);
        betaArray.push_back(Bk[3] + Bk[2]);
        betaArray.push_back(-Bk[2]/3);
        beta.push_back(betaArray);
    }
}



void interp::computeBasisFunctionsDerivatives(std::vector<std::vector<double>> &beta,
                                              std::vector<double> u,
                                              int order){
    // compute the derivatives of the basis spline function for evaluation of the
    // derivative of the grid-frield
    for (int i=0; i<dimension; i++){
        double Bk[4];
        for (int k=0; k<4; k++){
            Bk[k] = binomialCoefficient(order,k)*k*pow(u[i],k-1)*(order-k)*pow(1-u[i],order-k-1);
        }
        std::vector<double> betaArray;
        betaArray.push_back(Bk[0] + Bk[1]);
        betaArray.push_back(Bk[1]/3);
        betaArray.push_back(Bk[3] + Bk[2]);
        betaArray.push_back(-Bk[2]/3);
        beta.push_back(betaArray);
    }
}



double interp::binomialCoefficient(int n, int k){
    int nFac = 1, kFac = 1, nMkFac = 1;
    for (int i=1; i<=n; i++){
        nFac *= i;
    }
    for (int i=1; i<=k; i++){
        kFac *= i;
    }
    for (int i=1; i<=n-k; i++){
        nMkFac *= i;
    }
    return (double) (nFac)/(kFac*nMkFac);
}



double interp::getEval(std::vector<int> loc){
    // get the wrapping length evaluation given a vector 'loc' which contains, in
    // ascending dimension, the index in each dimension
    for (int i=0; i<evalsPair.size(); i++){
        if (evalsPair[i].first == loc){
            return evalsPair[i].second;
        }
    }
    return 0.0;
}



//    for (int i=0; i<dimension; i++){
//        std::vector<double> betaArray;
//        betaArray.push_back(2*pow(u[i],3) - 3*pow(u[i],2) + 1);
//        betaArray.push_back(pow(u[i],3) - 2*pow(u[i],2) + u[i]);
//        betaArray.push_back(-2*pow(u[i],3) + 3*pow(u[i],2));
//        betaArray.push_back(pow(u[i],3) - pow(u[i],2));
//        beta.push_back(betaArray);
//    }

//    for (int i=0; i<dimension; i++){
//        std::vector<double> betaArray;
//        betaArray.push_back(6*pow(u[i],2) - 6*u[i]);
//        betaArray.push_back(3*pow(u[i],2) - 4*u[i] + 1);
//        betaArray.push_back(-6*pow(u[i],2) + 6*u[i]);
//        betaArray.push_back(3*pow(u[i],2) - 2*u[i]);
//        beta.push_back(betaArray);
//    }