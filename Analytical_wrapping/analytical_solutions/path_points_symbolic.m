clear; close; clc;
format long
syms x Y hD H r yC yB real

% pythagorean lengths
d = sqrt(x^2 + (yC - yB)^2);
lT = sqrt(d^2 - r^2);

% angles
alpha = atan(r/lT);
gamma = atan((yC - yB)/x);
beta = alpha + gamma;

H = x*tan(beta);
y = x*((H-(yC - yB))/H);

theta = acos(r/y);

% desired
x_loc = r*cos(theta);
y_loc = (yC - yB) + r*sin(theta);

% taylor expansion
fx = x_loc + ...
    diff(x_loc,yB)*(Y-yB) + ...
    diff(x_loc,yB,2)*(Y-yB)^2;% + ...
    %diff(x_loc,hD,3)*(Y-hD)^3;
fy = y_loc + ...
    diff(y_loc,yB)*(Y-yB) + ...
    diff(y_loc,yB,2)*(Y-yB)^2;% + ...
    %diff(y_loc,hD,3)*(Y-hD)^3;
    
simplify(fx);
simplify(fy);

% polynomical terms
Cx = coeffs(fx,Y);
Cy = coeffs(fy,Y);

%% replacing
% from the c++ code
r = 0.08;
yC = 1.00;
yB = 0.5+0.1/2;
% hD = 1.00 - (0.5 + 0.1/2);
x = 0.5;

Cx = double(subs(Cx))
Cy = double(subs(Cy))

% syms r x yB yC hD real
x_loc = subs(x_loc);%,[r,x,yC],[0.08,0.5,1.00]);
y_loc = subs(y_loc);%,[r,x,yC],[0.08,0.5,1.00]);

%% plot approximation
xrange = 0:0.01:1;

CxPlot = Cx(3).*xrange.^2 + Cx(2).*xrange + Cx(1);
CyPlot = Cy(3).*xrange.^2 + Cy(2).*xrange + Cy(1);

NumxPlot = zeros(1,length(xrange));
NumyPlot = zeros(1,length(xrange));
for i = 1:length(xrange)
    NumxPlot(i) = double(subs(x_loc,yB,xrange(i)));
    NumyPlot(i) = double(subs(y_loc,yB,xrange(i)));
end

figure
subplot(1,2,1)
hold on
plot(xrange,CxPlot)
plot(xrange,NumxPlot)
legend("Taylor approx","Numerical sol")

subplot(1,2,2)
hold on
plot(xrange,CyPlot)
plot(xrange,NumyPlot)









