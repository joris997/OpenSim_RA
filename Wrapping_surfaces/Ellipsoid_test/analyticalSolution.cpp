/* -------------------------------------------------------------------------- *
 *                     analyticalSolution.cpp                                 *
 * -------------------------------------------------------------------------- *
 *  Computes the analytical solution of the muscle length for the case of     *
 *  two vertically moving objects with an offset from the cylinder wrapping.  *
 *  See "buildModel.cpp" for the construction of that model                   *
 *                                                                            *
 * Author(s): Joris Verhagen                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include "testCase.h"
#include <algorithm>

using namespace OpenSim;
using namespace SimTK;

double evaluateEquation(double& a, double& b, double& theta){
    return sqrt(pow(a*cos(theta),2) + pow(b*sin(theta),2));
}
double analyticalSolution(double centerObject, const testCase& tc, double &angleTangent){
    // create shorter notation from the global parameters
    double r = tc.CYLINDER_RADIUS;
    double x = tc.BODY_OFFSET;
    double h = centerObject;

    double lAnalytical;

    // use the analytical solution only for the non-rotated cylinder!
    if (h <= 0) {
        lAnalytical = 2*x;
        angleTangent = 0;
    }
    else {
        double hD = h - r;
        // distance center cylinder and connection point
        double d = sqrt(pow(x,2) + pow(hD,2));
        // length from muscle connection point to tangent circle
        double lTangent = sqrt(pow(d,2) - pow(r,2));

        double beta = asin(r/d);
        double alpha = atan(hD/x);
        double gamma = alpha + beta;

        double Y = x*tan(gamma);
        double H = Y - x*tan(alpha);

        // angle horizontal and perpendicular to tangent to circle
        angleTangent = acos(r/H);

        // total analytical length
        lAnalytical = 2*lTangent + 2*angleTangent*r;
    }
    return lAnalytical;
}
