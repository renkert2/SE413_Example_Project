function [F_p_f_sol,F_p_h_sol] = calcBodyForces(in1,in2,w_p)
%CALCBODYFORCES
%    [F_P_F_SOL,F_P_H_SOL] = CALCBODYFORCES(IN1,IN2,W_P)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-Aug-2021 12:20:03

%Input Arguments: r_COM_f, r_COM_h, r_l_2, w_p
r_COM_f1 = in1(1,:);
r_COM_f2 = in1(2,:);
r_COM_h1 = in2(1,:);
r_COM_h2 = in2(2,:);
t2 = r_COM_f1.*r_COM_h2;
t3 = r_COM_f2.*r_COM_h1;
t4 = -t3;
t5 = t2+t4;
t6 = 1.0./t5;
t7 = r_COM_f1.*r_COM_h1.*t6.*w_p;
F_p_f_sol = [-t7;t4.*t6.*w_p];
if nargout > 1
    F_p_h_sol = [t7;t2.*t6.*w_p];
end