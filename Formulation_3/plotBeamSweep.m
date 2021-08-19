function plotBeamSweep(X,S, opts)
arguments
    X (:,1) double
    S (:,1) struct
    opts.XLabel string = "X"
    opts.PlotOutputs string = ["beam_mass" "N_sigma_a" "N_sigma_bear" "N_lateral_buckling" "delta_max" "slip_margin" "tip_margin"]
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

