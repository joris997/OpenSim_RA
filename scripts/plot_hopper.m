clc; clear; close all;

data_hopper
       
figure
plot(pbp(:,1),pbp(:,2),'LineWidth',2)
hold on
plot(fbp5(:,1),fbp5(:,2))
plot(fbp10(:,1),fbp10(:,2))
grid on
legend('pbp','fbp5','fbp10')

rms5  = rms(pbp(:,2)-fbp5(:,2))
rms10 = rms(pbp(:,2)-fbp10(:,2))