clear; close all; clc;

%% 1D
if 0
    xrange = linspace(0,10,11);
    yvalues = zeros(1,length(xrange));
    for i = 1:length(xrange)
        yvalues(i) = xrange(i)^2;
    end

    figure
    plot(xrange,yvalues,'ro')
    hold on

    derivatives = zeros(1,length(xrange));
    for i = 1:length(xrange)
        if i==1 
            derivatives(i) = (yvalues(i+1)-yvalues(i))/(xrange(i+1)-xrange(i));
        elseif i==length(xrange)
            derivatives(i) = (yvalues(i)-yvalues(i-1))/(xrange(i)-xrange(i-1));
        else
            % use three-point difference because it does not desire uniform
            % interval lengths such as Catmull-Rom does.
            derivatives(i) = 1/2*((yvalues(i+1)-yvalues(i))/(xrange(i+1)-xrange(i)) + ...
                                  (yvalues(i)-yvalues(i-1))/(xrange(i)-xrange(i-1)));
        end
    end

    xLoc = 5.5;
    [~,i] = find((xLoc-xrange)>0,1,'last');

    t = (xLoc-xrange(i))/(xrange(i+1)-xrange(i));

    % Hermite basis functions expanded form
    h00 = 2*t^3 - 3*t^2 + 1;
    h10 = t^3 - 2*t^2 + t;
    h01 = -2*t^3 + 3*t^2;
    h11 = t^3 - t^2;
    % interpolation polynomial
    p = h00*yvalues(i) + h10*derivatives(i) + ...
        h01*yvalues(i+1) + h11*derivatives(i+1);

    plot(xLoc,p,'g*')
    title('Cubic hermite splines')


    %% general case
    figure
    plot(xrange,yvalues,'ro')
    hold on

    x = 5.5;        % desired interp location
    n = floor(x);   % lowest whole integer
    u = x-n;        % remaining fraction
    q = 4;          % number of interpolation points used
    g = (q-2)/2;    

    s_nq = 0;
    beta = [1/2*(u-1)^3*u*(2*u+1), ...
            -1/2*(u-1)*(6*u^4-9*u^3+2*u+2), ...
            1/2*u*(6*u^4-15*u^3+9*u^2+u+1), ...
            -1/2*(u-1)*u^3*(2*u-3)];
    for i = -g:1+g
        s_nq = s_nq + yvalues(i+n+1)*beta(i+2);
    end

    plot(x,s_nq,'b*')
    title('General case')

end


%%
xrange = linspace(0,10,21);
yvalues = zeros(length(xrange),length(xrange));
for i = 1:length(xrange)
    for ii = 1:length(xrange)
        yvalues(i,ii) = xrange(i)*xrange(ii);
    end
end
gridsize = xrange(2)-xrange(1);

figure
surf(xrange,xrange,yvalues)
hold on

x = [5.5;5.5];

arr1 = x(1) - xrange;
arr1(arr1<0) = nan;
[~,n1] = min(arr1);
arr2 = x(2) - xrange;
arr2(arr2<0) = nan;
[~,n2] = min(arr2);
n = [n1; n2];
u = [(x(1) - xrange(n1))/gridsize;
     (x(2) - xrange(n2))/gridsize];

q = 4;
g = (q-2)/2; 

beta = zeros(length(x),q);
for i = 1:length(x)
    beta(i,:) = [1/2*(u(i) - 1)^3*u(i)*(2*u(i) + 1), ...
                -1/2*(u(i) - 1)*(6*u(i)^4 - 9*u(i)^3 + 2*u(i) + 2), ...
                1/2*u(i)*(6*u(i)^4 - 15*u(i)^3 + 9*u(i)^2 + u(i) + 1), ...
                -1/2*(u(i) - 1)*u(i)^3*(2*u(i) - 3)];
end

s_nq = 0;
for i = -g:1+g
    for ii = -g:1+g
        Beta = beta(1,i+2)*beta(2,ii+2);            % +2 because g=-1 should be index[1]
        s_nq = s_nq + yvalues(i+n(1),ii+n(2))*Beta;
    end
end
s_nq
plot3(x(1),x(2),sum(s_nq),'r*')




% % beta
% betaProg = zeros(length(x),q);
% order = 3;
% for i = 1:length(x)
%     for ii = 0:q-1
%         bicoeff = factorial(order)/(factorial(ii)*factorial(order-ii));
%         betaProg(i,ii+1) = bicoeff*u(i)^ii*(1-u(i))^(order-ii);
%     end
% end



% s_nq = 0;
% for i = -g:1+g
%     Beta = 1;
%     for j = 1:length(x)
%         Beta = Beta*beta(j,i+2);
%     end
%     s_nq = s_nq + yvalues(i+n(1)+1,i+n(2)+1)*Beta;
% end
% plot3(x(1),x(2),sum(s_nq),'g*')



% beta = zeros(1,q);
% v = 5;          % order of the polynomial
% for i = 1:q
%     % use Bernstein basis polynomials to construct interp coeff
%     binoCoeff = factorial(v)/(factorial(q)*factorial(v-q))
%     beta(i) = binoCoeff*u^v*(1-u)^(n-v);
% end



% %% 2D
% global r 
% r = 0.1;
% 
% x1 = -0.3;
% x2 = -0.3;
% 
% discr = 20;
% y1 = linspace(-0.8,-0.2,discr);
% y2 = linspace(0.2,0.8,discr);
% 
% pathLengths = zeros(length(y1),length(y2));
% thetas = zeros(length(y1),length(y2));
% for ii = 1:length(y1)
%     for iv = 1:length(y2)
%         [~,pathLengths(ii,iv), thetas(ii,iv)] = ...
%                     returnLength([-0.3 y1(ii)],[-0.3,y2(iv)]);
%     end
% end
% 
% figure 
% surf(y1,y2,pathLengths)


% %% todo
% pathLengths = zeros(length(x1),length(y1),length(x2),length(y2));
% thetas = zeros(length(x1),length(y1),length(x2),length(y2));
% for i = 1:length(x1)
%     for ii = 1:length(y1)
%         for iii = 1:length(x2)
%             for iv = 1:length(y2)
%                 [~,pathLengths(i,ii,iii,iv), thetas(i,ii,iii,iv)] = ...
%                             returnLength([x1(i) y1(ii)],[x2(iii),y2(iv)]);
%             end
%         end
%     end
% end

% assume uniform parameter spacing, with the tangent as below we obtain
% Catmull-Rom splines which is a speical case of a cardinal spline

% tangents.x1x1 = zeros(length(x1),length(y1),length(x2),length(y2));
% tangents.yy = zeros(length(x1),length(y1),length(x2),length(y2));
% tangents.xy = zeros(length(x1),length(y1),length(x2),length(y2));
% tangents.yx = zeros(length(x1),length(y1),length(x2),length(y2));
% for i = 1:length(x1)
%     for ii = 1:length(y1)
%         for iii = 1:length(x2)
%             for iv = 1:length(y2)
%                try
%                    tangents.x1x1(i,ii,iii,iv) = (thetas(i+1,ii,iii,iv)-thetas(i-1,ii,iii,iv))/(x1(i+1)-x(i-1));
%                catch
%                    tangents.x1x1(i,ii,iii,iv) = 0;
%                end
%                
%                
%             end
%         end
%     end
% end


