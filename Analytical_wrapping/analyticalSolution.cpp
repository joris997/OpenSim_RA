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
#include "tests/test_a.h"

using namespace OpenSim;

double analyticalSolution(double leftHeight){
    // create shorter notation from the global parameters
    double s = BODY_SIZE;
    double r = CYLINDER_RADIUS;
    double x = BODY_OFFSET;
    double h = CYLINDER_HEIGHT;

    double lAnalytical, lCylinder;
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
    return lAnalytical;
}