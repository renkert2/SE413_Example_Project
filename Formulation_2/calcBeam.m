function S = calcBeam(Xs,P,M)
% Solve for Ladder Length
l_l = P.h_l / sin(Xs.theta_l);

% Beam Constant
[A_b,I_b] = beamProfile(Xs.w_b, Xs.h_b, Xs.t_b);
S.A_b = A_b;
S.I_b = I_b;

% Mass
S.beam_mass = beamMass(l_l, A_b, M.rho);

% Formulate optimization for r_p
[F_g, F_w, F_p] = solveStaticForces(l_l,Xs.theta_l, P.w_p, r_p);
[F_g_prime, ~, F_p_prime] = resolveForcesAlongBeam(Xs.theta_l, F_g, F_w, F_p);

% Stress
% - Axial Component
f_A = abs(F_g_prime(1));
sigma_A = (f_A/2)/A_b;

% - Bending Component
p = abs(F_p_prime(2)); % Component of force normal to beam
a = r_p;
b = l_l - r_p;

M_max = p*a*b / l_l;
sigma_B = M_max*(Xs.h_b/2)/I_b;

sigma = sigma_A + sigma_B;
S.N_sigma = M.sigma_max / sigma;

% Deflection
S.delta = (p*a*b*(a + 2*b)*sqrt(3*a*(a+2*b)))/(27*M.E*I_b*l_l); % Maximal Deflection

% Slippage!
S.slip_margin = P.mu_g * abs(F_g(2)) - abs(F_g(1));

% Tippage!
S.tip_margin = abs(F_w(1));
end

function [m] = beamMass(l_l, A_b, rho)
% mass of single ladder beam.  Rungs are omitted since they aren't part of the design study
m = l_l*A_b*rho;
end

function [F_g, F_w, F_p] = solveStaticForces(l,theta, w_p, r_p)
% w_p: Total Weight of Load
% r_p: Position of load along ladder, measured from ground

F_p = [0;-w_p/2];

F_g = [0;0];
F_w = [0;0];

F_w(1) = - r_p*(w_p/2)*cos(theta)/(l*sin(theta)); % Momement Balance about Ground
F_w(2) = 0; % Assume zero friction force at wall

F_g(1) = -F_w(1); % Horizontal Force Balance
F_g(2) = w_p/2; % Vertical Force Balance
end

function [F_g_prime, F_w_prime, F_p_prime] = resolveForcesAlongBeam(theta, F_g, F_w, F_p)
u_l = [cos(theta); sin(theta)];
projFun = @(F) dot(F,u_l)*u_l;
F_g_prime = projFun(F_g);
F_w_prime = projFun(F_w);
F_p_prime = projFun(F_p);
end

function [A_b,I_b] = beamProfile(w_b, h_b, t_b)
A_b = w_b*h_b - (w_b - 2*t_b)*(h_b - 2*t_b);
I_b = 1/12*(w_b*h_b^3 - (w_b - 2*t_b)*(h_b - 2*t_b)^3);
end
