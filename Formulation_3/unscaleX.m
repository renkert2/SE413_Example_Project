function X = unscaleX(Xs,X0)
%UNSCALEX Summary of this function goes here
%   Detailed explanation goes here
X = zeros(size(Xs));

X(1) = asin(Xs(1));
X(2:end) = Xs(2:end).*X0(2:end);
end

