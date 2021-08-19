function [F_p_f_sol, F_p_h_sol, g] = DeriveBodyForces()
syms r_COM_f r_COM_h [2 1] % Symbolic position vectors from COM to Foot, COM to Hand, and Knee to Foot. 

syms w_p % Variables/Parameters
syms f_f f_h_x f_h_y % Unknowns: Magnitude of Foot force vector, x and y components of hand force vector

F_COM = [0;-w_p]; % Force directed at person's Center of Mass
F_p_f = f_f*(-r_COM_f/norm(r_COM_f)); % Force of foot directed through COM
F_p_h = [f_h_x; f_h_y]; % Force of hand unknown

% Force Balance
eq_F = F_COM + F_p_f + F_p_h == [0;0];

% Moment Balance
eq_M = cross2D(r_COM_f, F_p_f) + cross2D(r_COM_h, F_p_h) == 0;

% Solve system of equations
[f_f_sol, f_h_x_sol, f_h_y_sol] = solve([eq_F; eq_M], [f_f; f_h_x; f_h_y], 'Real', true);

F_p_f_sol = subs(F_p_f, f_f, f_f_sol);
F_p_h_sol = subs(F_p_h, [f_h_x; f_h_y], [f_h_x_sol; f_h_y_sol]);

g = matlabFunction(F_p_f_sol, F_p_h_sol, 'Vars', {r_COM_f, r_COM_h, w_p}, 'File', 'calcBodyForces',...
    'Comments', "Input Arguments: r_COM_f, r_COM_h, r_l_2, w_p");


end