% Nomenclature:
% _l:ladder
% _b:beam profile
% _r:rung profile
% _g:Ground 
% _p:Person, applied load
% _w:Wall

%% Parameters
% Static Parameters
P = struct();
P.h_l = 5.69976; % m, https://vipertoolcompany.com/blogs/viper-tool-company-blog/how-high-are-the-gutters-on-a-house
P.mu_g = 0.9; % Rubber on Dry Asphalt, https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
P.r_r = 0.0254; % m, wild guess of 1" rung radius
P.w_p = 1334.47; % N, 300LBS Maximum ladder carrying capacity
P.r_p = 0.5; % Fractional position of load along ladder, measured from ground up

% Constraint Parameters
C.delta_max = 0.0254; %m, maximum allowable deflection
C.BearingSafetyFactor = 50;
C.MinStabilityForce = P.w_p / 10;
C.MaxRiskFactor = 100;

% Material Parameters - From https://www.engineeringtoolbox.com/engineering-materials-properties-d_1225.html
% E,rho,sigma_max
M = struct();
M(1).Mat = "GlassFilledEpoxy";
M(1).rho = 1.9e3; % Material Density, kg/m^3
M(1).E = 25e9; % Tensile Modulus, Pa
M(1).sigma_max = 300e6; % Tensile Strength, Pa

M(2).Mat = "1045Steel";
M(2).rho = 8.0e3;
M(2).E = 205e9;
M(2).sigma_max = 585e6;

M(3).Mat = "2045Aluminum";
M(3).rho = 2.7e3;
M(3).E = 73e9;
M(3).sigma_max = 450e6;

%% Design Variables:
% X = [l_l; theta_l; h_b; w_b; t_b]

l_b = [0,0,0,0.003175]';
u_b = [pi/2,Inf,Inf,Inf]';

theta_l_0 = deg2rad(45);
X0 = [theta_l_0, 0.2, 0.1, 0.01]'; % Start with large beam profile so constraints are satisfied
X0s = vec2struct(X0);
