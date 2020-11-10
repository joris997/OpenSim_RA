/* -------------------------------------------------------------------------- *
 *                     buildModel.cpp                                         *
 * -------------------------------------------------------------------------- *
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

using namespace OpenSim;

Model buildWrappingModel(const testCase& tc) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model
    auto model = Model();
    model.setName("EllipsoidWrapping");
    model.setGravity(Vec3(0));
    Ground& ground = model.updGround();

    // Create bodies for the top and bottom connection point
    double bodyMass = tc.BODY_MASS;
    double bodySideLength = tc.BODY_SIZE;
    auto inertia = bodyMass * Inertia::brick(Vec3(bodySideLength / 2.));

    auto bodyTop = new Body("bodyTop",bodyMass,Vec3(0),inertia);
    auto bodyBottom = new Body("bodyBottom",bodyMass,Vec3(0),inertia);
    auto bodyWrapping = new Body("bodyWrapping",bodyMass,Vec3(0),inertia);

    model.addBody(bodyTop);
    model.addBody(bodyBottom);
    model.addBody(bodyWrapping);

    // I should be able to use pin joint as hinges. Now the model works for a
    // cylinder or ellipsoid where the slider direction is perpendicular to the
    // spring spanning direction but this is kinda hacky. Seems like there is something
    // wrong with the PinJoint. Ask Ajay about this!
    auto pinJointOffset = Vec3(0,tc.BODY_OFFSET,0);
    auto pinTop = new PinJoint("pinTop",ground,pinJointOffset,Vec3(0),
                                        *bodyTop,Vec3(0),Vec3(0));
    auto pinBottom = new PinJoint("pinBottom",ground,-pinJointOffset,Vec3(0),
                                              *bodyBottom,Vec3(0),Vec3(0));

    // Add the joints to the model.
    model.addJoint(pinTop);
    model.addJoint(pinBottom);

    // Add the spring or muscle
    auto spring = new PathSpring("spring", tc.REST_LENGTH, tc.STIFFNESS, tc.DISSIPATION);
    spring->updGeometryPath().appendNewPathPoint("origin", *bodyTop, Vec3(0));
    spring->updGeometryPath().appendNewPathPoint("insertion", *bodyBottom, Vec3(0));

    // Add the moving frame on which the wrapping surface will be connected
    auto sliderGround = new SliderJoint("sliderGround",ground,Vec3(0),Vec3(0),
                                        *bodyWrapping,Vec3(0),Vec3(0));

    auto sine = new Sine(tc.S_AMPLITUDE,tc.S_OMEGA,tc.S_PHASE);
    auto& sliderCoord = sliderGround->updCoordinate();
    sliderCoord.setName("sliderX");
    sliderCoord.setPrescribedFunction(*sine);
    sliderCoord.setDefaultIsPrescribed(true);

    // Add the wrapping surfaces 'ellipsoid', 'cylinder', and 'sphere
    auto wrapSurface = new WrapCylinder();
    if (tc.WRAP_BODY_TYPE == "ellipsoid" || tc.WRAP_BODY_TYPE == "ellipse"){
        delete wrapSurface;
        auto wrapSurface = new WrapEllipsoid();
        wrapSurface->set_dimensions(Vec3(tc.CYLINDER_RADIUS,tc.CYLINDER_RADIUS,1));
    } else if (tc.WRAP_BODY_TYPE == "cylinder") {
        delete wrapSurface;
        auto wrapSurface = new WrapCylinder();
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
        wrapSurface->set_length(1);
    } else if (tc.WRAP_BODY_TYPE == "sphere") {
        delete wrapSurface;
        auto wrapSurface = new WrapSphere();
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
    } else {
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
        wrapSurface->set_length(1);
    }

    wrapSurface->set_xyz_body_rotation(tc.CYLINDER_ROT);
    wrapSurface->set_quadrant("+x");
    wrapSurface->setName("wrapSurface");

    bodyWrapping->addWrapObject(wrapSurface);

    spring->updGeometryPath().addPathWrap(*wrapSurface);

    model.addJoint(sliderGround);
    model.addForce(spring);

    // Visualization
    double sphereRadius = 0.04;
    auto bodyTopGeometry = new Sphere(sphereRadius);
    bodyTopGeometry->setColor(Vec3(0.8,0.1,0.1));
    bodyTop->attachGeometry(bodyTopGeometry);

    auto bodyBottomGeometry = new Sphere(sphereRadius);
    bodyBottomGeometry->setColor(Vec3(0.8,0.1,0.1));
    bodyBottom->attachGeometry(bodyBottomGeometry);

    auto bodyWrappingGeometry = new Sphere(sphereRadius);
    bodyWrappingGeometry->setColor(Vec3(0.1,0.8,0.1));
    bodyWrapping->attachGeometry(bodyWrappingGeometry);

    if (tc.SHOW_VISUALIZER) {
        model.setUseVisualizer(true);
    }
    return model;
}
