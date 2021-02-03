function returnModel = makeModel()
import org.opensim.modeling.*


model = Model();
% model = Model();
model.setUseVisualizer(false);

% Create two links, each with a mass of 1 kg, center of mass at the body's
% origin, and moments and products of inertia of zero.
humerus = Body('humerus', 1, Vec3(0), Inertia(0));
radius  = Body('radius',  1, Vec3(0), Inertia(0));

% Connect the bodies with pin joints. Assume each body is 1 m long.
shoulder = PinJoint('shoulder', ...
                    model.getGround(), ...  % Parent body
                    Vec3(0,2,0), ...        % Location in parent
                    Vec3(0), ...            % Orientation in parent
                    humerus, ...            % Child body
                    Vec3(0, 1, 0), ...      % Location in child
                    Vec3(0));               % Orientation in child
elbow = PinJoint('elbow', ...
                 humerus, Vec3(0), Vec3(0), radius, Vec3(0, 1, 0), Vec3(0));

% Add a muscle that flexes the elbow.
biceps = Millard2012EquilibriumMuscle('biceps', 200, 0.6, 0.55, 0);
biceps.addNewPathPoint('origin',    humerus, Vec3(0, 0.8, 0));
biceps.addNewPathPoint('insertion', radius,  Vec3(0, 0.7, 0));

% Add a controller that specifies the excitation of the muscle.
% brain = PrescribedController();
% brain.addActuator(biceps);
% Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
% brain.prescribeControlForActuator('biceps', StepFunction(0.5, 3, 0.3, 1));

% Add components to the model.
model.addBody(humerus);    
model.addBody(radius);
model.addJoint(elbow);
model.addJoint(shoulder);  
model.addForce(biceps);
% model.addController(brain);

% Add display geometry.
bodyGeometry = Ellipsoid(0.1, 0.5, 0.1);
bodyGeometry.setColor(Vec3(0.5));  % Gray

% Attach an ellipsoid to a frame located at the center of each body.
humerusCenter = PhysicalOffsetFrame();
humerusCenter.setName('humerusCenter');
humerusCenter.setParentFrame(humerus);
humerusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
humerus.addComponent(humerusCenter);
humerusCenter.attachGeometry(bodyGeometry.clone());

radiusCenter = PhysicalOffsetFrame();
radiusCenter.setName('radiusCenter');
radiusCenter.setParentFrame(radius);
radiusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
radius.addComponent(radiusCenter);
radiusCenter.attachGeometry(bodyGeometry.clone());

state = model.initSystem();

% Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, true);
elbow.getCoordinate().setValue(state, 0.5 * pi);
model.equilibrateMuscles(state);

returnModel = model;
end