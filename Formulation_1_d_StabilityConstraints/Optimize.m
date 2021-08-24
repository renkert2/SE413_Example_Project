%% Optimization
opts = optimoptions('fmincon','Display','iter-detailed', 'PlotFcn', {'optimplotx', 'optimplotfval', 'optimplotfirstorderopt'});
[X_opt,~,exitflag,output,lambda,grad,hessian] = fmincon(@(X) objfun(X,P,M(1)), X0, A, b,[],[],l_b,u_b,@(X) nlcon(X,P,M(1),C), opts);
X_opt_s = vec2struct(X_opt);

function f = objfun(X,P,M)
    Xs = vec2struct(X);
    S = calcBeam(Xs,P,M);
    f = 2*S.beam_mass;
end

function [c,ceq] = nlcon(X,P,M,C)
c = zeros(5,1);

Xs = vec2struct(X);
S = calcBeam(Xs,P,M);

% Bending
c(1) = S.delta - C.delta_max;
c(2) = 1 - S.N_sigma;

% Slippage!
c(3) = 0 - S.slip_margin; 

% Slippage!
c(4) = C.MinStabilityForce-S.slip_margin; 

% Tippage!
c(5) = C.MinStabilityForce-S.tip_margin; 

ceq = [];   
end
