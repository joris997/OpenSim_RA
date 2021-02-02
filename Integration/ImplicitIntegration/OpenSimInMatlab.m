%% Import Java libraries
import org.opensim.modeling.*

arm = Model();
arm.setName('bicep_curl');
arm.setUseVisualizer(true);

% ---------------------------------------------------------------------------
% Create two links, each with a mass of 1 kg, centre of mass at the body's
% origin, and moments and products of inertia of zero.
% ---------------------------------------------------------------------------

humerus = Body('humerus',...
                    1.0,...
                    Vec3(0),...
                    Inertia(0, 0, 0));
radius = Body('radius',...
                   1.0,...
                   Vec3(0),...
                   Inertia(0, 0, 0));

% ---------------------------------------------------------------------------
% Connect the bodies with pin joints. Assume each body is 1m long.
% ---------------------------------------------------------------------------

shoulder = PinJoint('shoulder',...
                         arm.getGround(),... % PhysicalFrame
                         Vec3(0),...
                         Vec3(0),...
                         humerus,... % PhysicalFrame
                         Vec3(0, 1, 0),...
                         Vec3(0));

elbow = PinJoint('elbow',...
                      humerus,... % PhysicalFrame
                      Vec3(0),...
                      Vec3(0),...
                      radius,... % PhysicalFrame
                      Vec3(0, 1, 0),...
                      Vec3(0));

% ---------------------------------------------------------------------------
% Add a muscle that flexes the elbow (actuator for robotics people).
% ---------------------------------------------------------------------------

biceps = Millard2012EquilibriumMuscle('biceps',...  % Muscle name
                                           200.0,...  % Max isometric force
                                           0.6,...  % Optimal fibre length
                                           0.55,...  % Tendon slack length
                                           0.0);  % Pennation angle
biceps.addNewPathPoint('origin',...
                       humerus,...
                       Vec3(0, 0.8, 0));

biceps.addNewPathPoint('insertion',...
                       radius,...
                       Vec3(0, 0.7, 0));

% ---------------------------------------------------------------------------
% Add a controller that specifies the excitation of the muscle.
% ---------------------------------------------------------------------------

brain = PrescribedController();
brain.addActuator(biceps);
brain.prescribeControlForActuator('biceps',...
                                  StepFunction(0.5, 3.0, 0.3, 1.0));

% ---------------------------------------------------------------------------
% Build model with components created above.
% ---------------------------------------------------------------------------

arm.addBody(humerus);
arm.addBody(radius);
arm.addJoint(shoulder); % Now required in OpenSim4.0
arm.addJoint(elbow);
arm.addForce(biceps);
arm.addController(brain);

% ---------------------------------------------------------------------------
% Add a console reporter to print the muscle fibre force and elbow angle.
% ---------------------------------------------------------------------------

% We want to write our simulation results to the console.
reporter = ConsoleReporter();
reporter.set_report_time_interval(1.0);
reporter.addToReport(biceps.getOutput('fiber_force'));
elbow_coord = elbow.getCoordinate().getOutput('value');
reporter.addToReport(elbow_coord, 'elbow_angle');
arm.addComponent(reporter);

% ---------------------------------------------------------------------------
% Add display geometry. 
% ---------------------------------------------------------------------------

bodyGeometry = Ellipsoid(0.1, 0.5, 0.1);
bodyGeometry.setColor(Vec3(0.5)); % Gray
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

% ---------------------------------------------------------------------------
% Configure the model.
% ---------------------------------------------------------------------------

state = arm.initSystem();
% Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, true);
elbow.getCoordinate().setValue(state, 0.5 * pi);
arm.equilibrateMuscles(state);

% ---------------------------------------------------------------------------
% Configure the visualizer
% ---------------------------------------------------------------------------

viz = arm.updVisualizer().updSimbodyVisualizer();
viz.setBackgroundColor(Vec3(0)); % white
viz.setGroundHeight(-2)

% ---------------------------------------------------------------------------
% Simulate.
% ---------------------------------------------------------------------------

manager = Manager(arm);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(10.0);