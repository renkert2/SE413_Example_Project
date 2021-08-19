function S = calcBeam(Xs,P,M)
% Beam Constant
[A_b,I_b] = beamProfile(Xs.w_b, Xs.h_b, Xs.t_b);
S.A_b = A_b;
S.I_b = I_b;

% Mass
S.beam_mass = beamMass(Xs.l_l, A_b, M.rho);

[F_g, ~, F_p] = solveStaticForces(Xs.l_l,Xs.theta_l, P.w_p);
F_p_prime = F_p(2)*cos(Xs.theta_l); % Component of force perpendicular to beam

% Bending
S.delta = abs(F_p_prime(2)*(Xs.l_l)^3/(48*M.E*I_b)); % Deflection at center

M_max = abs(F_p_prime(2)*Xs.l_l/4); % Bending moment
sigma = M_max*(Xs.h_b/2)/I_b; % Bending stress
S.N_sigma = M.sigma_max / sigma; % Safety Factor

% Slippage!
S.slip_margin = P.mu_g * abs(F_g(2)) - abs(F_g(1));
end

function [m] = beamMass(l_l, A_b, rho)
% mass of single ladder beam.  Rungs are omitted since they aren't part of the design study
m = l_l*A_b*rho;
end

function [F_g, F_w, F_p] = solveStaticForces(l, theta, w_p)
% w_p: Total Weight of Load
% r_p: Position of load along ladder, measured from ground

F_p = [0;-w_p/2];

F_g = [0;0];
F_w = [0;0];

F_w(1) = - (l/2)*(w_p/2)*cos(theta)/(l*sin(theta)); % Momement Balance about Ground
F_w(2) = 0; % Assume zero friction force at wall

F_g(1) = -F_w(1); % Horizontal Force Balance
F_g(2) = w_p/2; % Vertical Force Balance
end

function [A_b,I_b] = beamProfile(w_b, h_b, t_b)
A_b = w_b*h_b - (w_b - 2*t_b)*(h_b - 2*t_b);
I_b = 1/12*(w_b*h_b^3 - (w_b - 2*t_b)*(h_b - 2*t_b)^3);
end
