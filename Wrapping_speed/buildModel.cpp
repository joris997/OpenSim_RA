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
    bool showVisualizer = tc.SHOW_VISUALIZER;
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model
    auto model = Model();
    model.setName("Normal_Wrapping");
    model.setGravity(Vec3(0, -9.80665, 0));



    // BODIES
    double bodyMass1 = tc.BODY_MASS;
    double bodyMass2 = tc.BODY_MASS*tc.BODY_MASS_FACTOR;
    double bodySideLength = tc.BODY_SIZE;
    auto bodyInertia1 = bodyMass1 * Inertia::brick(Vec3(bodySideLength / 2.));
    auto bodyInertia2 = bodyMass2 * Inertia::brick(Vec3(bodySideLength / 2.));
    auto bodyLeft = new Body("bodyLeft", bodyMass1, Vec3(0), bodyInertia1);
    auto bodyRight = new Body("bodyRight", bodyMass2, Vec3(0), bodyInertia2);
    // Create static ground body for wrapping surface frame
    auto bodyGround = new Body("bodyGround", 1, Vec3(0), bodyInertia1);

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
    sliderCoordLeft.setDefaultValue(tc.BODY_HEIGHT);
    auto &sliderCoordRight =
            sliderRight->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoordRight.setName("yCoordSliderRight");
    sliderCoordRight.setDefaultValue(tc.BODY_HEIGHT);



    // SPRING
    auto spring = new PathSpring("spring", tc.REST_LENGTH, tc.STIFFNESS, tc.DISSIPATION);
    spring->updGeometryPath().appendNewPathPoint("origin", *bodyLeft, Vec3(0, bodySideLength / 2, 0));
    spring->updGeometryPath().appendNewPathPoint("insertion", *bodyRight, Vec3(0, bodySideLength / 2, 0));



    // WRAPPING SURFACE
    auto wrappingFrame = new PhysicalOffsetFrame("wrappingFrame", model.getGround(),
                                                 SimTK::Transform(Vec3(0, tc.CYLINDER_HEIGHT, 0)));

    auto wrapSurface = new WrapCylinder();
    if (tc.WRAP_BODY_TYPE == "ellipsoid"){
        delete wrapSurface;
        auto wrapSurface = new WrapEllipsoid();
        wrapSurface->set_dimensions(Vec3(tc.CYLINDER_RADIUS,tc.CYLINDER_RADIUS,1));
    } else {
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
        wrapSurface->set_length(1);
    }
    wrapSurface->set_xyz_body_rotation(tc.CYLINDER_ROT);
    wrapSurface->set_quadrant("+y");
    wrapSurface->setName("wrapSurface");

    wrappingFrame->addWrapObject(wrapSurface);
    bodyGround->addComponent(wrappingFrame);

    spring->updGeometryPath().addPathWrap(*wrapSurface);

    model.addForce(spring);


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






/*
auto system = SimTK::MultibodySystem{};
auto forces = SimTK::GeneralForceSubsystem{system};
auto cables = SimTK::CableTrackerSubsystem{system};

auto cable = SimTK::CablePath{
        cables,
        *bodyLeft, Vec3(0,bodySideLength/2, 0),
        *bodyRight, Vec3(0, bodySideLength/2, 0),
};
SimTK::CableSpring cable2(forces,cable,tc.REST_LENGTH,tc.STIFFNESS,tc.DISSIPATION);

auto wrapSurface = new SimTK::ContactGeometry::Ellipsoid(Vec3(tc.CYLINDER_RADIUS,tc.CYLINDER_RADIUS,1));
auto obstacleSurface = SimTK::CableObstacle::Surface{
        cable,
        model.getGround(),
        SimTK::Transform{SimTK::Rotation{}, Vec3{0,0,0}},
        wrapSurface,
};
obstacleSurface.setContactPointHints(Vec3{-tc.CYLINDER_RADIUS, 0.001, 0.0},Vec3{tc.CYLINDER_RADIUS, 0.001, 0.0});
*/
