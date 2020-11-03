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
double analyticalSolution(double leftHeight, const testCase& tc, bool useNumerical=false){
    // create shorter notation from the global parameters
    double s = tc.BODY_SIZE;
    double r = tc.CYLINDER_RADIUS;
    double x = tc.BODY_OFFSET;
    double h = tc.CYLINDER_HEIGHT;
    Vec3 rot = tc.CYLINDER_ROT;

    double lAnalytical, lCylinder;
    // Use the numerical variant for computing the rotated cylinder wrapping length
    if (useNumerical) {
        if (leftHeight+s/2 >= h+r) {
            lAnalytical = 2*x;
        }
        else {
            // Including rotation
            double a = (r/cos(rot[0]));
            double b = (r/cos(rot[1]));

            double hD = h - (leftHeight + s / 2);

            double yc = -hD;
            double yp = (a*pow(b,2) + (pow(b,2)*x*(a*yc*sqrt(- pow(a,2)*pow(b,2) + pow(a,2)*pow(yc,2) + pow(b,2)*pow(x,2)) - a*pow(b,2)*x))
                                      /(pow(a,2)*pow(yc,2) + pow(b,2)*pow(x,2)))/(a*yc);
            double xp = a*sqrt(1-(pow(yp,2)/pow(b,2)));
            // length from muscle connection point to tangent circle
            double lTangent = sqrt(pow(x-xp,2) + pow(-hD-yp,2));
            // angle from vertical to the point on the ellipse
            double theta = atan(xp/yp);

            int nSteps = 1000;
            double thetaK1, thetaK2, thetaK3, thetaK4, k1, k2, k3, k4, arcLength = 0;
            double nTheta = theta/nSteps;
            for (int i=0; i<nSteps; i++) {
                thetaK1 = i*nTheta;
                k1 = evaluateEquation(a,b,thetaK1);
                thetaK2 = thetaK1+0.5*nTheta*k1;
                k2 = evaluateEquation(a,b,thetaK2);
                thetaK3 = thetaK1+0.5*nTheta*k2;
                k3 = evaluateEquation(a,b,thetaK3);
                thetaK4 = thetaK1+nTheta*k3;
                k4 = evaluateEquation(a,b,thetaK4);
                arcLength += (nTheta/6)*(k1+2*k2+2*k3+k4);
            }
            lAnalytical = 2*arcLength + 2*lTangent;
        }

    }
    // use the analytical solution only for the non-rotated cylinder!
    else {
        if (leftHeight+s/2 >= h+r) {
            lAnalytical = 2*x;
        }
        else {
            double hD = h - (leftHeight + s / 2);
            // distance center cylinder and connection point
            double d = sqrt(pow(x, 2) + pow(hD, 2));
            // length from muscle connection point to tangent circle
            double lTangent = sqrt(pow(d, 2) - pow(r, 2));
            // angle connection point to horizontal
            double beta = atan(r / lTangent) + atan(hD / x);
            // height of extension of connection point to tangent circle (h + small part)
            double H = tan(beta) * x;
            // center of cylinder, horizontal line until it touches muscle
            double y = ((H - hD) / H) * x;
            // angle horizontal and perpendicular to tangent to circle
            double theta = acos(r / y);
            // length over cylinder part
            lCylinder = (SimTK::Pi - 2 * theta) * r;
            // total analytical length
            lAnalytical = 2 * lTangent + lCylinder;
        }
    }
    return lAnalytical;
}
