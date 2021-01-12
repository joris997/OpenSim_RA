function [length,pathLength,theta] = returnLength(p1,p2)
global r

%% 3rd quadrant
l1Tangent = sqrt(norm(p1,2)^2 - r^2);

beta1 = atan(r/l1Tangent) + atan(abs(p1(1))/abs(p1(2)));

H1 = tan(beta1)*abs(p1(2));

% y1 = r - (H1 - sin(beta1)*l1Tangent);
% theta1 = acos(y1/r);
y1 = H1 - abs(p1(1));
theta1 = acos(r/y1);

l1Cylinder = theta1*r;

l1 = l1Tangent + l1Cylinder;

%% 2nd quadrant
l2Tangent = sqrt(norm(p2,2)^2 - r^2);

beta2 = atan(r/l2Tangent) + atan(abs(p2(1))/abs(p2(2)));

H2 = tan(beta2)*abs(p2(2));

% y2 = r - (H2 - sin(beta2)*l2Tangent);
y2 = H2 - abs(p2(1));
theta2 = acos(r/y2);
% theta2 = acos(y2/r);

l2Cylinder = theta2*r;

l2 = l2Tangent + l2Cylinder;


length = l1 + l2;
pathLength = l1Cylinder + l2Cylinder;
theta = theta1 + theta2;
end

% 
% hD = h - leftHeight;
% % distance center cylinder and connection point
% d = sqrt(pow(x, 2) + pow(hD, 2));
% % length from muscle connection point to tangent circle
% lTangent = sqrt(pow(d, 2) - pow(r, 2));
% % angle connection point to horizontal
% beta = atan(r / lTangent) + atan(hD / x);
% % height of extension of connection point to tangent circle (h + small part)
% H = tan(beta) * x;
% % center of cylinder, horizontal line until it touches muscle
% y = ((H - hD) / H) * x;
% % angle horizontal and perpendicular to tangent to circle
% theta = acos(r / y);
% % length over cylinder part
% lengthCylinder = (pi - 2 * theta) * r;
% % total analytical length
% length = 2 * lTangent + lengthCylinder;