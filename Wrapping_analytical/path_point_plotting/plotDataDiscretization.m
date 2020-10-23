clear; close; clc;

data
timeInterval = 0.05;
timeArray = linspace(0,length(results(1).muscleLength)*timeInterval,length(results(1).muscleLength));

times = [];
npoints = [];
for i = 1:length(results)
    times = [times results(i).time];
    npoints = [npoints results(i).npoints];
end
times = [times wResults.time];
npoints = [npoints wResults.npoints];

%% plot muscle length
figure
subplot(2,2,1)
hold on
for i = 1:length(results)
    plot(timeArray,results(i).muscleLength)
end

plot(timeArray,wResults.muscleLength,'r')
grid on
xlabel('time [s]')
ylabel('muscle length [m]')

%% plot computation time
subplot(2,2,2)
hold on
for i = 1:length(times)
    h = bar(npoints(i),times(i));
    if npoints(i)==0
        set(h,'FaceColor','r')
    else
        set(h,'FaceColor','b')
    end
end    
grid on
xlabel('number of discretization points')
ylabel('simulation time [s]')

%% compute percentage fiber length
optFiberLength = 0.55;
subplot(2,2,3)
hold on
for i = 1:length(results)
    plot(timeArray,results(i).fiberLength./optFiberLength)
end
plot(timeArray,wResults.fiberLength./optFiberLength,'r')
grid on
xlabel('time [s]')
ylabel('fiberLength normalized w.r.t. optFiberLength')

%% compute RMS
rms = zeros(1,length(results));
for i=1:length(results)
    rms(i) = sum(sqrt((results(i).muscleLength - wResults.muscleLength).^2));
end
subplot(2,2,4)
bar(npoints(1:end-1),rms)
grid on
xlabel('number of discretization points')
ylabel('rse [m]')
