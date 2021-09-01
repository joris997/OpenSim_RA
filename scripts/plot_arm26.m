clc; clear; close all;

data_arm26

muscles = {'TRIlong','TRIlat','TRImed','BIClong','BICshort','BRA'}';

figure
for i = 1:6
    subplot(2,3,i)
    plot(pbp(:,1),pbp(:,i+1),'LineWidth',2)
    hold on
    plot(fbp5(:,1),fbp5(:,i+1))
    plot(fbp10(:,1),fbp10(:,i+1))
    title(muscles(i))
    grid on
end


rms5 = zeros(6,1);
rms10 = zeros(6,1);
for i = 1:6
    rms5(i) = rms(pbp(:,i+1)-fbp5(:,i+1));
    rms10(i) = rms(pbp(:,i+1)-fbp10(:,i+1));
end

table(muscles,rms5,rms10)