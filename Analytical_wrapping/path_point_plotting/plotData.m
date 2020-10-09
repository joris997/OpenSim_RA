clear; close; clc;

data

%% plot muscle length
times = [];
npoints = [];
figure
subplot(1,3,1)
hold on
for i = 1:length(results)
    plot(results(i).points)
    times = [times results(i).time];
    npoints = [npoints results(i).npoints];
end
times = [times wResults.time];
npoints = [npoints wResults.npoints];

plot(wResults.points,'r')
grid on
xlabel('simulation time instance [-]')
ylabel('muscle length [m]')


%% plot computation time
subplot(1,3,2)
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


%% compute RMS
rms = zeros(1,length(results));
for i=1:length(results)
    rms(i) = sum(sqrt((results(i).points - wResults.points).^2));
end
subplot(1,3,3)
bar(npoints(1:end-1),rms)
grid on
xlabel('number of discretization points')
ylabel('rmse [m]')
