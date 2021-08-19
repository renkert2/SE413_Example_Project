function Xs = vec2struct(X)
    Xs = struct();
    Xs.l_l = X(1);
    Xs.theta_l = X(2);
    Xs.h_b = X(3);
    Xs.w_b = X(4);
    Xs.t_b = X(5);
end