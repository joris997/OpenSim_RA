clear; close; clc;

syms t

n = 3;
Bk = sym(zeros(1,4));
for i = 0:3
    Bk(i+1) = binomialCoefficient(n,i)*t^i*(1-t)^(n-i);
end

h00 = Bk(1) + Bk(2)
h10 = 1/3*Bk(2)
h01 = Bk(4) + Bk(3)
h11 = -1/3*Bk(3)


%% plotting
h00_ = zeros(1,100);
h10_ = zeros(1,100);
h01_ = zeros(1,100);
h11_ = zeros(1,100);
for i=0:99
    h00_(i+1) = double(subs(h00,t,i/100));
    h10_(i+1) = double(subs(h10,t,i/100));
    h01_(i+1) = double(subs(h01,t,i/100));
    h11_(i+1) = double(subs(h11,t,i/100));
end

xrange = linspace(0,1,100);
figure
plot(xrange,h00_)
hold on
plot(xrange,h10_)
plot(xrange,h01_)
plot(xrange,h11_)
legend('h00','h10','h01','h11')



%% functions
function bc = binomialCoefficient(n, k)
bc = factorial(n)/(factorial(k)*factorial(n-k));
end