function Xs = scaleX(X,X0)
%SCALE Summary of this function goes here
%   Detailed explanation goes here
Xs = zeros(size(X));

Xs(1) = sin(X(1));
Xs(2:end) = X(2:end)./X0(2:end);
end

