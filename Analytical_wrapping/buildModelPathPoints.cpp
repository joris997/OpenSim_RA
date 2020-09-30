/* -------------------------------------------------------------------------- *
 *                     buildModelPathPoints.cpp                               *
 * -------------------------------------------------------------------------- *
 * Test_suite that checks analytical solutions of muscle lengths over         *
 * wrapping surfaces with the numerical solutions of OpenSim. Especially      *
 * important for unconventional wrapping (over a rotated cylinder for example *
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
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include "testCase.h"

using namespace OpenSim;

Model buildWrappingModelPathPoints(bool showVisualizer, const testCase& tc) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model on earth.
    auto model = Model();
    model.setName("Normal_Wrapping");
    model.setGravity(Vec3(0, -9.80665, 0));



    // BODIES
    double bodyMass = 30.0;
    double bodySideLength = tc.BODY_SIZE;
    auto bodyInertia = bodyMass * Inertia::brick(Vec3(bodySideLength / 2.));
    auto bodyLeft = new Body("bodyLeft", bodyMass, Vec3(0), bodyInertia);
    auto bodyRight = new Body("bodyRight", bodyMass, Vec3(0), bodyInertia);
    // Create static ground body for wrapping surface frame
    auto bodyGround = new Body("bodyGround", 1, Vec3(0), bodyInertia);

    model.addBody(bodyLeft);
    model.addBody(bodyRight);
    model.addBody(bodyGround);

    // Attach the pelvis to ground with a vertical slider joint, and attach the
    // pelvis, thigh, and shank bodies to each other with pin joints.
    Vec3 sliderOrientation(0, 0, SimTK::Pi / 2.);
    Vec3 bodyOffset(tc.BODY_OFFSET, 0, 0);
    auto sliderLeft = new SliderJoint("sliderLeft", model.getGround(), bodyOffset,
                                      sliderOrientation, *bodyLeft, Vec3(0), sliderOrientation);
    auto sliderRight = new SliderJoint("sliderRight", model.getGround(), -bodyOffset,
                                       sliderOrientation, *bodyRight, Vec3(0), sliderOrientation);
    auto weldGround = new WeldJoint("weldGround", model.getGround(), *bodyGround);

    // Add the joints to the model.
    model.addJoint(sliderLeft);
    model.addJoint(sliderRight);
    model.addJoint(weldGround);

    // Set the coordinate names and default values. Note that we need "auto&"
    // here so that we get a reference to the Coordinate rather than a copy.
    auto &sliderCoordLeft =
            sliderLeft->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoordLeft.setName("yCoordSliderLeft");
    sliderCoordLeft.setDefaultValue(0.5);
    auto &sliderCoordRight =
            sliderRight->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoordRight.setName("yCoordSliderRight");
    sliderCoordRight.setDefaultValue(0.5);



    // MUSCLES AND SPRINGS
    double mclFmax = 4000., mclOptFibLen = 0.55, mclTendonSlackLen = 0.5,
            mclPennAng = 0.;
    auto muscle = new Thelen2003Muscle("muscle", mclFmax, mclOptFibLen,
                                       mclTendonSlackLen, mclPennAng);

    auto springToLeft = new PointToPointSpring(model.getGround(), Vec3(0),
                                               *bodyLeft, Vec3(0, -bodySideLength / 2, 0), 100, 0.5);
    springToLeft->setName("springToLeft");
    auto springToRight = new PointToPointSpring(model.getGround(), Vec3(0),
                                                *bodyRight, Vec3(0, -bodySideLength / 2, 0), 100, 0.5);
    springToRight->setName("springToRight");



    // WRAPPING SURFACE
    auto wrappingFrame = new PhysicalOffsetFrame("wrappingFrame", model.getGround(),
                                                 SimTK::Transform(Vec3(0, tc.CYLINDER_HEIGHT, 0)));
    bodyGround->addComponent(wrappingFrame);
    // Add the wrapping surface
    muscle->addNewPathPoint("origin", *bodyLeft, Vec3(0, bodySideLength / 2, 0));
    auto center_point = Vec3(0,0,0);
    double r = tc.CYLINDER_RADIUS;
    double p0 = SimTK::Pi/4;
    double pe = 3*SimTK::Pi/4;
    int nPoints = 6;
    double increment = (pe-p0)/(nPoints);
    for (int i=0; i<nPoints+1; i++) {
        std::string name = "sub" + std::to_string(i);
        muscle->addNewPathPoint(name, *wrappingFrame, center_point + Vec3(r * cos(p0), r * sin(p0), 0));
        p0 += increment;
    }
    auto *movingTest = new MovingPathPoint();
    movingTest->setName("movingTest");
    movingTest->setParentFrame(*wrappingFrame);
    SimTK::Vector a;
    a[0] = 1; a[1] = 2; a[2] = 3;
    PolynomialFunction xFunc(a);
    movingTest->set_x_location(xFunc);

    muscle->addNewPathPoint("insertion", *bodyRight, Vec3(0, bodySideLength / 2, 0));

    model.addForce(muscle);
    model.addForce(springToLeft);
    model.addForce(springToRight);



    // CONTROLLER
    auto brain = new PrescribedController();
    brain->setActuators(model.updActuators());
    double t[5] = {0.0, 1.0, 2.0, 3.0, 4.0}, x[5] = {0.0, 1.0, 0.0, 0.5, 0.0};
    auto controlFunction = new PiecewiseConstantFunction(5, t, x);
    brain->prescribeControlForActuator("muscle", controlFunction);
    model.addController(brain);

    // Attach geometry to the bodies and enable the visualizer.
    auto bodyLeftGeometry = new Brick(Vec3(bodySideLength / 2.));
    bodyLeftGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    bodyLeft->attachGeometry(bodyLeftGeometry);

    auto bodyRightGeometry = new Brick(Vec3(bodySideLength / 2.));
    bodyRightGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    bodyRight->attachGeometry(bodyRightGeometry);

    if (showVisualizer)
        model.setUseVisualizer(true);

    return model;
}