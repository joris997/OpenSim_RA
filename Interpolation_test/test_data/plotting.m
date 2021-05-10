clear; close all; clc;

%%
fbp_40_arm

figure
subplot(2,2,1)
plot(pbp.lengthening_speed(:,1),pbp.lengthening_speed(:,2))
hold on
plot(fbp.lengthening_speed(:,1),fbp.lengthening_speed(:,2))
legend('pbp','fbp')
title('Arm 40 lengthening speed')

subplot(2,2,2)
plot(pbp.length(:,1),pbp.length(:,2))
hold on
plot(fbp.length(:,1),fbp.length(:,2))
legend('pbp','fbp')
title('Arm 40 length')

%
fbp_80_arm

subplot(2,2,3)
plot(pbp.lengthening_speed(:,1),pbp.lengthening_speed(:,2))
hold on
plot(fbp.lengthening_speed(:,1),fbp.lengthening_speed(:,2))
legend('pbp','fbp')
title('Arm 80 lengthening speed')

subplot(2,2,4)
plot(pbp.length(:,1),pbp.length(:,2))
hold on
plot(fbp.length(:,1),fbp.length(:,2))
legend('pbp','fbp')
title('Arm 80 length')

%%
fbp_5_hopper

figure
subplot(2,2,1)
plot(pbp.lengthening_speed(:,1),pbp.lengthening_speed(:,2))
hold on
plot(fbp.lengthening_speed(:,1),fbp.lengthening_speed(:,2))
legend('pbp','fbp')
title('Hopper 5 lengthening_speed')

subplot(2,2,2)
plot(pbp.length(:,1),pbp.length(:,2))
hold on
plot(fbp.length(:,1),fbp.length(:,2))
legend('pbp','fbp')
title('Hopper 5 length')

%
fbp_20_hopper

subplot(2,2,3)
plot(pbp.lengthening_speed(:,1),pbp.lengthening_speed(:,2))
hold on
plot(fbp.lengthening_speed(:,1),fbp.lengthening_speed(:,2))
legend('pbp','fbp')
title('Hopper 20 lengthening_speed')

subplot(2,2,4)
plot(pbp.length(:,1),pbp.length(:,2))
hold on
plot(fbp.length(:,1),fbp.length(:,2))
legend('pbp','fbp')

title('Hopper 20 length')
