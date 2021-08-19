function [c,ceq] = nlcon(Xs,X0,P,M,C)
c = zeros(6,1);
ceq = [];

X = unscaleX(Xs,X0);
Xs = vec2struct(X);

S = calcBeam(Xs,P,M);

% Compressive Stress
c(1) = 1 - S.N_sigma_a; % Safety factor greater than 1

% Bending
c(2) = (S.delta_max - C.delta_max)/C.delta_max;

% Bearing
c(3) = 1 - S.N_sigma_bear;

% Lateral Buckling
c(4) = 1 - S.N_lateral_buckling;

% Safety!

% Tip Margin
c(5) = (C.MinStabilityForce - S.tip_margin)/C.MinStabilityForce; 

% Slip Margin
c(6) = (C.MinStabilityForce - S.slip_margin)/C.MinStabilityForce;
end
