function [q, qs, qss, qsss, interval] = pathProgressEvaluateCubic(nodes, coefficients, progress)
%PATHPROGRESSEVALUATECUBIC Evaluate array coefficients and spatial derivatives.
% Internal knots belong to the right interval, s=1 to the last interval.
% Each output is joints-by-query, except the 1-by-query interval indices.

n = numel(nodes);
if n < 2 || ~isrow(nodes) || ~isreal(nodes) || any(~isfinite(nodes)) || ...
        nodes(1) ~= 0 || nodes(end) ~= 1 || any(diff(nodes) <= 0) || ...
        size(coefficients,1) < 1 || size(coefficients,2) ~= n-1 || ...
        size(coefficients,3) ~= 4 || ndims(coefficients) ~= 3 || ...
        ~isreal(coefficients) || any(~isfinite(coefficients(:)))
    error('PathProgress:InvalidPolynomial','Invalid cubic nodes or coefficients.');
end
if ~isvector(progress) || ~isreal(progress) || any(~isfinite(progress(:))) || ...
        any(progress(:) < nodes(1) | progress(:) > nodes(end))
    error('PathProgress:OutsideDomain','Progress queries must lie in [0,1].');
end

dof = size(coefficients,1);
q = zeros(dof,numel(progress));
qs = q;
qss = q;
qsss = q;
interval = zeros(1,numel(progress));
for i = 1:numel(progress)
    value = progress(i);
    left = 1;
    right = n;
    while right-left > 1
        middle = floor((left+right)/2);
        if value < nodes(middle)
            right = middle;
        else
            left = middle;
        end
    end
    interval(i) = left;
    u = value-nodes(left);
    for joint = 1:dof
        a = coefficients(joint,left,1);
        b = coefficients(joint,left,2);
        c = coefficients(joint,left,3);
        d = coefficients(joint,left,4);
        q(joint,i) = ((d*u+c)*u+b)*u+a;
        qs(joint,i) = (3*d*u+2*c)*u+b;
        qss(joint,i) = 6*d*u+2*c;
        qsss(joint,i) = 6*d;
    end
end
end
