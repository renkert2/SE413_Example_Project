init;

opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'DiffMinChange', 1e-3, 'Display','iter-detailed', 'PlotFcn', {'optimplotx', 'optimplotfval', 'optimplotfirstorderopt'});
As = repmat(unscaleX(ones(size(X0)),X0)',size(A,1),1).*A;

stability_force_range = linspace(10,P.Body.w_p,10);
for i = 1:numel(stability_force_range)
    
    C.MinStabilityForce = stability_force_range(i);
    sol(i).MinStabilityForce = C.MinStabilityForce;
    
    [sol(i).X_opt_s,sol(i).Mass,sol(i).exitflag,sol(i).output,sol(i).lambda,sol(i).grad,sol(i).hessian] = fmincon(@(Xs) objfun(Xs,X0,P,M(1)), scaleX(X0,X0), As, b,[],[],scaleX(l_b, X0),scaleX(u_b, X0),@(Xs) nlcon(Xs,X0,P,M(1),C), opts);
    sol(i).X_opt = unscaleX(sol(i).X_opt_s, X0);

    % sol(i).S = calcBeam(sol(i).X_opt_struct, P, M);
    
    X0 = sol(i).X_opt;
end

