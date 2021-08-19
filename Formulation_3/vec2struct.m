function Xs = vec2struct(X)
    Xs = struct();
    Xs.theta_l = X(1);
    Xs.h_b = X(2);
    Xs.w_b = X(3);
    Xs.t_b = X(4);
end