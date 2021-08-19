function S = calcBeam(Xs,P,M,r_p_frac)
% Xs: Solution Structure
% P: Parameter Structure
% M: Material Structure
% r_p_frac: Fractional position of body along ladder, varies from 0 to 1.

% Constants
g = 9.81; % m/s^2

% Solve for Ladder Length
l_l = P.h_l / sin(Xs.theta_l);
S.l_l = l_l;

% Distance from base of ladder to wall
w_l = l_l*cos(Xs.theta_l);
S.w_l = w_l;

% Beam Constant
[A_b,I_b_x,I_b_y] = beamProfile(Xs.w_b, Xs.h_b, Xs.t_b);
S.A_b = A_b;
S.I_b_x = I_b_x;
S.I_b_y = I_b_y;

% Mass
S.beam_mass = beamMass(l_l, A_b, M.rho);

% Position Dependent factors
if nargin == 3
    % Compressive Stress
    [r] = fminbnd(@(r) calcBeamInner(r).N_sigma_a, 0.1, .9);
    s = calcBeamInner(r);
    S.S_N_sigma_a = s;
    S.N_sigma_a = s.N_sigma_a;
    
    % Bending
    [r] = fminbnd(@(r) -(calcBeamInner(r).delta_max), 0.1, 0.9);
    s = calcBeamInner(r);
    S.S_delta_max = s;
    S.delta_max = s.delta_max;
    
    % Safety
    s = calcBeamInner(1); % Parameter sweep revealed this to be 'worst case scenario'
    S.S_r_p_max = s;
    S.slip_margin = s.slip_margin;
    S.tip_margin = s.tip_margin;
    
    % Lateral Buckling
    s = s;
    S.N_lateral_buckling = s.N_lateral_buckling;
    
    % Bearing
    s = s; % Use previous solution; bearing stress independent of r_p
    S.N_sigma_bear = s.N_sigma_bear;
elseif nargin == 4
    S = calcBeamInner(r_p_frac,S);
end

    function S = calcBeamInner(r_p_frac, S)
        if nargin == 1
            S = struct();
        end
        
        r_p = r_p_frac*(l_l - abs(P.Body.r_COM_f(1))); % Maximum position along ladder is when COM is aligned with top of ladder
        S.r_p_frac = r_p_frac;
        S.r_p = r_p;

        [F_g, F_w, F_p_f, F_p_h] = solveStaticForces(l_l, Xs.theta_l, r_p, P);
        S.F_g = F_g;
        S.F_w = F_w;
        S.F_p_f = F_p_f;
        S.F_p_h = F_p_h;
        
        [F_g_prime, F_w_prime, F_p_f_prime, F_p_h_prime] = resolveForcesAlongBeam(Xs.theta_l, F_g, F_w, F_p_f, F_p_h);
        S.F_g_prime = F_g_prime;
        S.F_w_prime = F_w_prime;
        S.F_p_f_prime = F_p_f_prime;
        S.F_p_h_prime = F_p_h_prime;
        
        % Array of Internal Forces
        beam_forces = [F_g_prime, F_p_f_prime, F_p_h_prime, F_w_prime];
        beam_forces_r = [0, r_p, r_p + norm(P.Body.r_f_h), l_l];
        
        % Axial Force F(x)
        % S.F = @(x) -F_g_prime(1)*(0 <= x & x<r_p) + F_w_prime(1)*(x>=r_p);
        S.F = @(x) -sum(beam_forces(1,:).*(x >= beam_forces_r));
        
        % Shear Forces S(x)
        S.S = @(x) sum(beam_forces(2,:).*(x >= beam_forces_r));
        
        % Calculate Moment and Deflection using Beam-Column Theory - Worse Near r_p = .5
        [S.BVPSol, S.delta_max, S.M] = SolveBVP(S.F, S.S, M.E, I_b_x, P.Body.w_p/2, r_p, l_l, Xs.theta_l, w_l);
        
        % Calculate Axial Stress - Worse Near r_p = .5
        [S.AxialStressSol,S.sigma_a_max] = calcAxialStress(S.M,S.F,Xs.h_b/2,I_b_x,A_b,l_l);
        S.N_sigma_a = M.sigma_max / S.sigma_a_max;
        
        % Bearing Stress - Constant
        S.sigma_bear = P.Body.w_p/(2*P.r_r*Xs.t_b);
        S.N_sigma_bear = M.sigma_max / S.sigma_bear;
        
        % Lateral Buckling - Worse at r_p = 1
        F_A_cr = pi^2*M.E*I_b_y / l_l^2;
        F_A = F_g_prime(1); % Worst case scenario
        S.N_lateral_buckling = F_A_cr / F_A;
        
        % Safety!
        
        % Slippage!
        S.slip_margin = P.mu_g * abs(F_g(2)) - abs(F_g(1));
        
        % Tippage!
        S.tip_margin = -F_w(1);
    end
end

function [s,delta_max,M] = SolveBVP(F, S, E, I, P, a, l, theta, w)
opts = bvpset();
xmesh = linspace(0,l,1000);
solinit = bvpinit(xmesh, @guess);
sol = bvp4c(@odefun,@bcfun,solinit, opts);

% Re-interpolate for better resolution
s.x = xmesh;
s.y = interp1(sol.x',sol.y',xmesh','pchip')'; %[delta; delta'; delta'']
M = @(x) interp1(s.x,s.y(3,:),x)*E*I;
s.M = M;

% Correct Delta from Beam Shortening
d_x_corr = sqrt(diff(s.x).^2 - diff(s.y(1,:)).^2);
x_corr = s.x(1) + [0 cumsum(d_x_corr)];
l_corr = sqrt((x_corr(end))^2 + s.y(1,end)^2);
d_theta = - asin(s.y(1,end)/l_corr);
theta_corr = acos(w/l_corr) + d_theta;

R = RotationMatrix(theta_corr);
pos = zeros(2,numel(x_corr));
for i = 1:numel(x_corr)
    pos(:,i) = R*[x_corr(i); s.y(1,i)];
end

x_pos = pos(1,:);
y_0 = [pos(1,:).*tan(theta)];
y_pos = pos(2,:);

delta_pos = RotationMatrix(-theta)*pos;
delta_corr = delta_pos(2,:);

delta_max = max(abs(delta_corr));

s.x_pos = x_pos;
s.y_0 = y_0;
s.y_pos = y_pos;
s.delta_max = delta_max;

    function dydx = odefun(x,y)
        dydx = zeros(3,1);
        dydx(1) = y(2);
        dydx(2) = y(3);
        dydx(3) = (1/(E*I))*(S(x)+F(x)*y(2));
    end
    function res = bcfun(ya, yb)
        res = [ya(1); ya(3); yb(3)];
    end
    function y = guess(x)
        b = l - a;
        
        % Case when x <= a
        y1 = [-(P*b*x / (6*E*I*l)).*(l^2 - b^2 - x.^2);
            (P*b*(b^2 - l^2 + 3*x.^2))/(6*E*I*l);
            P*b*x/(l*E*I)];
        % Case when x > a
        y2 = [-(P*a*(l-x) / (6*E*I*l)).*(2*l*x - a^2 - x.^2);
            -(P*a*(a^2+2*l^2-6*l*x+3*x.^2))/(6*E*I*l);
            P*a*(l-x)/(l*E*I)];
        y = y1.*(x <= a) + y2.*(x>a);
    end
end

function [s, sigma_max] = calcAxialStress(M,F,c,I,A,L)
    [x,fval] = fminbnd(@(x) -abs(calcAxialStress_(x)),0,L);
    
    s.sigma_max = abs(fval);
    sigma_max = s.sigma_max;
    
    s.x_sigma_max = x;
    s.sigma = @calcAxialStress_;
    
    function sigma = calcAxialStress_(x)
        sigma = (-M(x)*c)/I + F(x)/A;
    end     
end

function [m] = beamMass(l_l, A_b, rho)
% mass of single ladder beam.  Rungs are omitted since they aren't part of the design study
m = l_l*A_b*rho;
end

function [F_g, F_w, F_p_f, F_p_h] = solveStaticForces(l,theta, r_p, P)
% l: Length of ladder
% theta: angle of ladder
% w_p: Total Weight of Person
% r_p: Distance along ladder from ground to foot

R = RotationMatrix(theta);
r_COM_f = R*P.Body.r_COM_f;
r_COM_h = R*P.Body.r_COM_h;
r_f_h = R*P.Body.r_f_h;
r_g_f = r_p*[cos(theta);sin(theta)];
r_g_h = r_g_f + r_f_h;

% Calculate Body Forces
[F_p_f,F_p_h] = calcBodyForces(r_COM_f, r_COM_h, P.Body.w_p); % Forces of Ladder on Body
F_p_f = -F_p_f / 2; 
F_p_h = -F_p_h / 2; % Forces of Body on Ladder, distributed evenly between each beam

M_p_f = cross2D(r_g_f, F_p_f);
M_p_h = cross2D(r_g_h, F_p_h);

F_w = [0;0];
F_w(1) = - (M_p_f + M_p_h)/(- l*sin(theta)); % Momement Balance about Ground
F_w(2) = 0; % Assume zero friction force at wall

F_g = -(F_w + [0; -P.Body.w_p]);
end

function varargout = resolveForcesAlongBeam(theta, varargin)
x_prime = [cos(theta); sin(theta)];
y_prime = [cos(theta+pi/2); sin(theta+pi/2)];
projFun = @(F) [dot(F,x_prime); dot(F,y_prime)];

N = numel(varargin);
varargout = cell(size(varargin));
for i = 1:N
    varargout{i} = projFun(varargin{i});
end
end

function [A_b,I_b_x,I_b_y] = beamProfile(w_b, h_b, t_b)
A_b = w_b*h_b - (w_b - 2*t_b)*(h_b - 2*t_b);
I_b_x = 1/12*(w_b*h_b^3 - (w_b - 2*t_b)*(h_b - 2*t_b)^3);
I_b_y = 1/12*(h_b*w_b^3 - (h_b - 2*t_b)*(w_b - 2*t_b)^3);
end
