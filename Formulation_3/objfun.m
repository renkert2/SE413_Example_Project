function f = objfun(Xs,X0,P,M)
    X = unscaleX(Xs,X0);
    Xs = vec2struct(X);

    
    S = calcBeam(Xs,P,M);
    f = 2*S.beam_mass;
end