%% Import Java libraries
import org.opensim.modeling.*

%% Instantiate model
modelDir = 'models/';
importModel = true;
% models = {'arm26.osim',
%           'double_pendulum.osim',
%           'gait10dof18musc.osim',
%           'gait2354_simbody.osim',
%           'gait2392_millard2012muscle.osim',
%           'Rajagopal2015.osim',
%           'SoccerKickingModel.osim',
%           'ToyLandingModel.osim',
%           'Tug_of_War.osim'};
models = {'ToyLandingModel.osim'};
          
odeNames = {'ode45','ode23','ode113', ...
            'ode15s','ode23s','ode23t','ode23tb'};
%     model = Model(fullfile(modelDir,'arm26.osim'));
%     model = Model('arm26.osim');
%     model = Model('double_pendulum.osim');
%     model = makeModel();
simtimes = zeros(length(models),length(odeNames));
for modeli = 1:length(models)
%     model = Model('gait10dof18musc.osim')
    model = Model(models{modeli});

    tAvg = zeros(1,length(odeNames));
    for i = 1:length(odeNames)
        integratorName = odeNames{i};
        integratorOptions = odeset('AbsTol', 1e-4);
        controlsFuncHandle = [];
        tf = 1;

        disp(['Starting ', integratorName, ' simulation'])

        time = 0;
        nSim = 1;
        for ii = 1:nSim
            % Reconfigure the model.
            state = model.initSystem();

            tStart = tic;
            states = IntegrateOpenSimPlant(model,controlsFuncHandle, ...
                [0 tf],integratorName, integratorOptions);
            toc;
            time = time + toc;
        end
        tAvg(i) = time/nSim;
    end

    simtimes(modeli,:) = tAvg;
%     [tAvg,idx] = sort(tAvg);
%     odeNames = odeNames(idx);
%     for i = 1:length(odeNames)
%         disp(['simtime ',odeNames{i},9,num2str(tAvg(i))])
%     end

end







%% old code
% plotting = false;
% if plotting
%     disp(['Plotting results'])
%     PlotOpenSimData(states,'time', {'/jointset/elbow/elbow_coord_0/value'})
% end


% % Add a console reporter to print the muscle fiber force and elbow angle. The
% % output will be written to the log file (out.log) in the current directory.
% reporter = ConsoleReporter();
% reporter.set_report_time_interval(1.0);
% reporter.addToReport(biceps.getOutput('fiber_force'));
% reporter.addToReport(elbow.getCoordinate().getOutput('value'), 'elbow_angle');
% model.addComponent(reporter);


% %% Simulate with MATLAB ode's
% % use methods()
% % model.getMultibodySystem.toString;
% model.getNumBodies;
% model.getNumCoordinates;
% % model.updMatterSubsystem.calcSystemJacobian;
% % model.updMatterSubsystem.multiplyByG;
% % model.updMatterSubsystem.multiplyByM;
% Ekin = model.updMatterSubsystem.calcKineticEnergy(state);
% Msys = model.updMatterSubsystem.calcSystemMass(state);
% 
% % EoM:
% % M ddq = G(q) + V(q,dq) + S(q,dq) + R f
% % M = opensim.modeling.Matrix(4);
% % model.updMatterSubsystem.calcM(state,zeros(4,4));
% 
% % model.setStateVariableValues;
% 
% % just trying to get a vector and see what it looks like
% someVec = model.updMatterSubsystem.calcSystemMassCenterVelocityInGround(state);
% osimVec3ToArray(someVec);


% manager = Manager(model);
% finalTime = 10;
% manager.initialize( state );
% 
% disp(['Starting OpenSim simulation'])
% manager.integrate( finalTime );