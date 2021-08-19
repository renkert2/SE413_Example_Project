%% Position of Mass Study
r_p_range = linspace(0.01,.99,50);
for i = 1:numel(r_p_range)
    S_r_p(i) = calcBeam(X0s,P,M(1), r_p_range(i));
end
%%
plotBeamSweep(r_p_range, S_r_p, 'XLabel', "r_p")
% At the bottom of the ladder, you are more likely to tip because less force is exerted on the wall. 
% As you climb the ladder, this force increases and you become more likely to slip.  

%% Ladder Angle Study
theta_range = linspace(0.01,pi/2 - 0.01,50);
Xs = X_opt_struct;
for i = 1:numel(theta_range)
    Xs.theta_l = theta_range(i);
    try
        S_theta(i) = calcBeam(Xs,P,M(1));
    catch
        ;
    end
end

%%
plotBeamSweep(theta_range, S_theta, 'XLabel', "theta_l")