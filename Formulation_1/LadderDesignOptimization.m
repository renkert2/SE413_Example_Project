%% Design Variables:
% X = [l_l; theta_l; h_b; w_b; t_b]

l_b = [P.h_l,0,0,0,0]';
u_b = [Inf,pi/2,Inf,Inf,Inf]';

theta_l_0 = deg2rad(45);
l_l_0 = P.h_l/sin(theta_l_0);
x0 = [l_l_0, theta_l_0, 0.2, 0.1, 0.01]'; % Start with large beam profile so constraints are satisfied
X0s = vec2struct(X0);
%% Linear Constraints
% Ax <= b
% 1: h_b >= 2*r_r + 2*t_b
% 2: w_b >= 2*t_b

A = [0,0,-1,0,2;...
    0,0,0,-1,2];
b = [-2*P.r_r;0];

%% Optimization
opts = optimoptions('fmincon','Display','iter-detailed', 'PlotFcn', {'optimplotx', 'optimplotfval', 'optimplotfirstorderopt'});
[X_opt,~,exitflag,output,lambda,grad,hessian] = fmincon(@(X) objfun(X,P,M(1)), X0,A,b,[],[],l_b,u_b,@(X) nlcon(X,P,M(1),C), opts);
X_opt_s = vec2struct(X_opt);

function f = objfun(X,P,M)
    Xs = vec2struct(X);
    S = calcBeam(Xs,P,M);
    f = 2*S.beam_mass;
end

function [c,ceq] = nlcon(X,P,M,C)
c = zeros(7,1);

Xs = vec2struct(X);
S = calcBeam(Xs,P,M);

% Bending
c(1) = S.delta - C.delta_max;
c(2) = 1 - S.N_sigma;

% Slippage!
c(3) = C.MinStabilityForce-S.slip_margin; 

% Equality Constraints
ceq = zeros(1,1);

% Geometry constraint ensuring the ladder reaches the correct height
ceq(1) = Xs.l_l*sin(Xs.theta_l) - P.h_l;    
end
