%% Position of Mass Study
r_p_range = linspace(0,1,50);
for i = 1:numel(r_p_range)
    P.r_p = r_p_range(i);
    S_r_p(i) = calcBeam(X0s,P,M(1));
end

plotBeamSweep(r_p_range, S_r_p, 'XLabel', "r_p", 'PlotOutputs', ["f_A", "slip_margin", "tip_margin"])
% At the bottom of the ladder, you are more likely to tip because less force is exerted on the wall. 
% As you climb the ladder, this force increases and you become more likely to slip.  

%% Ladder Angle Study
theta_range = linspace(0,pi/2,50);
Xs = X0s;
for i = 1:numel(theta_range)
    Xs.theta_l = theta_range(i);
    S(i) = calcBeam(Xs,P,M(1));
end

plotBeamSweep(theta_range, S, 'XLabel', "theta_l", 'PlotOutputs', ["slip_margin", "tip_margin", "delta"])