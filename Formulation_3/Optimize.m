opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'DiffMinChange', 1e-3, 'Display','iter-detailed', 'PlotFcn', {'optimplotx', 'optimplotfval', 'optimplotfirstorderopt'});

As = repmat(unscaleX(ones(size(X0)),X0)',size(A,1),1).*A;
[X_opt_s,~,exitflag,output,lambda,grad,hessian] = fmincon(@(Xs) objfun(Xs,X0,P,M(1)), scaleX(X0,X0), As, b,[],[],scaleX(l_b, X0),scaleX(u_b, X0),@(Xs) nlcon(Xs,X0,P,M(1),C), opts);
X_opt = unscaleX(X_opt_s, X0);
X_opt_struct = vec2struct(X_opt);

