clear; clc;
syms yp a b xc yc

%% compute point on ellipse
assume(a>0 & b>0 & xc>0 & yp>0)

% from rewriting the eclipse equation
xp = a*sqrt(1-(yp^2/b^2));

% tangent of ellipse is parallel to slope of the linear line
eq = (b^2/a^2)*(xp/yp) == (yp - yc)/(xp-xc);

sym_sol = solve(eq,yp)%,'ReturnConditions',true)

a = 0.04;
b = 0.04;
xc = 0.1;
yc = -1.0;