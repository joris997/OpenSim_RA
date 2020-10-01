clear; close; clc;
format long
syms x Y hD H r real

% pythagorean lengths
d = sqrt(x^2 + hD^2);
lT = sqrt(d^2 - r^2);

% angles
alpha = atan(r/lT);
gamma = atan(hD/x);
beta = alpha + gamma;

H = x*tan(beta);
y = x*((H-hD)/H);

theta = acos(r/y);

% desired
x_loc = r*cos(theta);
y_loc = hD + r*sin(theta);

% taylor expansion
fx = x_loc + ...
    diff(x_loc,hD)*(Y-hD) + ...
    diff(x_loc,hD,2)*(Y-hD)^2;% + ...
    %diff(x_loc,hD,3)*(Y-hD)^3;
fy = y_loc + ...
    diff(y_loc,hD)*(Y-hD) + ...
    diff(y_loc,hD,2)*(Y-hD)^2;% + ...
    %diff(y_loc,hD,3)*(Y-hD)^3;
    
simplify(fx);
simplify(fy);

% polynomical terms
Cx = coeffs(fx,Y);
Cy = coeffs(fy,Y);

%% replacing
% from the c++ code
r = 0.08;
hD = 1.00 - (0.5 + 0.1/2);
x = 0.5;

Cx = double(subs(Cx))
Cy = double(subs(Cy))








