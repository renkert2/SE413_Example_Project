function plotBeamSweep(X,S, opts)
arguments
    X (:,1) double
    S (:,1) struct
    opts.XLabel string = "X"
    opts.PlotOutputs string = ["A_b" "I_b" "beam_mass" "f_A" "sigma_A" "f_A_buckle" "delta" "sigma_B" "sigma_bear" "slip_margin" "tip_margin"]
end
fields = opts.PlotOutputs;
t = tiledlayout(numel(fields),1,'TileSpacing','compact');

for f = fields
    nexttile
    plot(X,vertcat(S.(f)))
    ylabel(f);
    if f == fields(end)
        xlabel(opts.XLabel);
    end
end

