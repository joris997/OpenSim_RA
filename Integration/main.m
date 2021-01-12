clear; close; clc;
%% dynamical model
f = @(t,y) [y(1)^2 - y(1)^3];

%% some testing of different MATLAB ode's
t0 = 0;
tf = 500;
delta = 0.0001;
opts = odeset('RelTol',1e-4);

figure
subplot(1,2,1)
start_time = clock();
ode45(f,[t0,2/delta],delta,opts);
runtime45 = etime(clock(),start_time);

subplot(1,2,2)
start_time = clock();
ode15s(f,[t0,2/delta],delta,opts);
runtime15s = etime(clock(),start_time);

% delta = 0.0001:
% t15s = 0.39, t45 = 2.67 -> 6.85x
% delta = 0.00001:
% t15s = 0.36, t45 = 15.36 -> 42.67x

%% forward and backward euler methods
g = 9.81;
l = 1;
f = @(t,y) [y(2); -g/l*sin(y(1))];

% explicit
hE = 0.01;
tf = 5;
y = [pi/8; 0];
stepsE = ceil(tf/hE);
resultExplicit = zeros(2,stepsE);
for i = 1:stepsE
    y = y + hE*f(0,y);
    resultExplicit(:,i) = y;
end

% implicit
syms y1 y2 real
f = [y2; -g/l*sin(y1)];

hI = 0.01;
y = [pi/8; 0];
stepsI = ceil(tf/hI);
resultImplicit = zeros(2,stepsI);
for i = 1:stepsI
    yAns = solve([y1; y2] == y + hI*f,[y1 y2]);
    y = [double(yAns.y1); double(yAns.y2)];
    resultImplicit(:,i) = y;
end

figure
plot(linspace(0,tf,stepsE),resultExplicit(1,:),'b')
hold on
% plot(linspace(0,tf,stepsE),resultExplicit(2,:),'b-')

plot(linspace(0,tf,stepsI),resultImplicit(1,:),'r')
% plot(linspace(0,tf,stepsI),resultImplicit(2,:),'r-')
grid on

%% diagonal runge kutta compared to rk4 scheme
% https://en.wikipedia.org/wiki/List_of_Runge%E2%80%93Kutta_methods#Kutta's_third-order_method
% explicit RK2
f = @(t,y) [y(2); -g/l*sin(y(1))];

h = 0.01;
y = [pi/8; 0];
stepsRK2 = ceil(tf/h);
resultRK2 = zeros(2,stepsRK2);

b = [0, 1/2];
c = [0; 1];
a = [0, 0;
     1/2, 0];

for i = 1:stepsRK2
    k1 = f(0,y);
    k2 = f(0,y+0.5*h*k1);
    k3 = f(0,y+3*h*k2/4);
%     y = y + h*(1/2*k2);
    y = y+h/9*(2*k1+3*k2+4*k3);

    resultRK2(:,i) = y;
end
plot(linspace(0,tf,stepsRK2),resultRK2(1,:),'k')

% diagonal RK
h = 0.01;
y = [pi/8; 0];
stepsRKI = ceil(tf/h);
resultRKI = zeros(2,stepsRKI);

b = [-1/2, 3/2];
c = [1/2, 3/2];
a = [1/2, 0;
     -1/2, 2];
 
syms k11 k12 k21 k22 real
for i = 1:stepsRKI
    yInput1 = y + h*a(1,1)*[k11;k12] + h*a(1,2)*[k21;k22];
    eqEval1 = [yInput1(2); -g/l*sin(yInput1(1))];
    
    yInput2 = y + h*a(2,1)*[k11;k12] + h*a(2,2)*[k21;k22];
    eqEval2 = [yInput2(2); -g/l*sin(yInput2(1))];
    
    eq1 = [k11;k12] == eqEval1;
    eq2 = [k21;k22] == eqEval2;
    
    kSol = solve([eq1;eq2],[k11;k12;k21;k22]);
    
    y = y + h*(b(1)*double([kSol.k11;kSol.k12]) + b(2)*double([kSol.k21;kSol.k22]));
    
    resultRKI(:,i) = y;
%     eq1 = [k11;k12] == f(t + c(1)*h, y + h*a(1,1)*[k11;k12] + h*a(1,2)*[k21;k22]);
%     eq2 = [k21;k22] == f(t + c(2)*h, y + h*a(2,1)*[k11;k12] + h*a(2,2)*[k21;k22]);
%     kSol = solve([eq1; eq2],[k1 k2]);
%     y = y + h*(b(1)*kSol.k1 + b(2)*kSol.k2);
%     t = t + h;
end

plot(linspace(0,tf,stepsRKI),resultRKI(1,:),'g')
% plot(linspace(0,tf,stepsRKI),resultRKI(2,:),'g-')
legend('explicit Euler','implicit Euler','explicit RK2','implicit RK2')
