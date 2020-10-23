clear; close; clc;

data
timeInterval = results(1).interval;
timeArray = linspace(0,length(results(1).muscleLength)*timeInterval,length(results(1).muscleLength));

times = [];
npoints = [];
muscleMaxForces = [];
optFiberLengths = [];
tendonSlackLengths = [];
for i = 1:length(results)
    times = [times results(i).time];
    npoints = [npoints results(i).npoints];
    muscleMaxForces = [muscleMaxForces results(i).muscleMaxForce];
    optFiberLengths = [optFiberLengths results(i).optFiberLength];
    tendonSlackLengths = [tendonSlackLengths results(i).tendonSlackLength];    
end

%% compute percentage fiber length
subplot(1,2,1)
hold on
for i = 1:length(results)
    plot(timeArray,results(i).fiberLength./optFiberLengths(i))
end
grid on
xlabel('time [s]')
ylabel('fiberLength normalized w.r.t. optFiberLength')

%% plot computation time
subplot(1,2,2)
hold on
for i = 1:length(times)
    h = bar(tendonSlackLengths(i),times(i));
    set(h,'BarWidth',0.02)
end    
grid on
xlabel('tendon slack length')
ylabel('simulation time [s]')


