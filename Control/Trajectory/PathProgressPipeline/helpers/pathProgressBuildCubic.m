function coefficients = pathProgressBuildCubic(nodes, qNodes)
%PATHPROGRESSBUILDCUBIC Build a not-a-knot C2 spline in normalized progress.
% nodes: 1-by-N, strictly increasing from 0 to 1. qNodes: joints-by-N [rad].
% coefficients(j,k,:) = [a,b,c,d] for a + b*u + c*u^2 + d*u^3,
% where u = s - nodes(k). Shape: joints-by-(N-1)-by-4.

n = numel(nodes);
if n < 4 || ~isrow(nodes) || ~isreal(nodes) || any(~isfinite(nodes)) || ...
        nodes(1) ~= 0 || nodes(end) ~= 1 || any(diff(nodes) <= 0) || ...
        ~ismatrix(qNodes) || isempty(qNodes) || size(qNodes,2) ~= n || ...
        ~isreal(qNodes) || any(~isfinite(qNodes(:)))
    error('PathProgress:InvalidNodes','Expected >=4 distinct nodes from 0 to 1 and finite joint data.');
end
h = diff(nodes);
m = n-2;
lower = zeros(1,m);
diagonal = zeros(1,m);
upper = zeros(1,m);
rhs = zeros(size(qNodes,1),m);
for k = 1:m
    left = h(k);
    right = h(k+1);
    lower(k) = left;
    diagonal(k) = 2*(left+right);
    upper(k) = right;
    rhs(:,k) = 6*((qNodes(:,k+2)-qNodes(:,k+1))/right - ...
        (qNodes(:,k+1)-qNodes(:,k))/left);
end

% Equal third derivatives across the first and last interior knots.
% Eliminate the two endpoint curvatures before the tridiagonal solve.
firstRatio = h(1)/h(2);
lastRatio = h(end)/h(end-1);
diagonal(1) = diagonal(1) + h(1)*(1+firstRatio);
upper(1) = upper(1) - h(1)*firstRatio;
diagonal(m) = diagonal(m) + h(end)*(1+lastRatio);
lower(m) = lower(m) - h(end)*lastRatio;

% Thomas elimination. Reject pivots small relative to their row scale.
rowScale = abs(lower)+abs(diagonal)+abs(upper);
for k = 1:m
    if ~isfinite(diagonal(k)) || abs(diagonal(k)) <= 32*eps*rowScale(k) || ...
            any(~isfinite(rhs(:,k)))
        error('PathProgress:InvalidPivot','Ill-conditioned cubic system at interior row %d.',k);
    end
    if k < m
        multiplier = lower(k+1)/diagonal(k);
        diagonal(k+1) = diagonal(k+1)-multiplier*upper(k);
        rhs(:,k+1) = rhs(:,k+1)-multiplier*rhs(:,k);
    end
end
curvature = zeros(size(qNodes));
curvature(:,n-1) = rhs(:,m)/diagonal(m);
for k = m-1:-1:1
    curvature(:,k+1) = (rhs(:,k)-upper(k)*curvature(:,k+2))/diagonal(k);
end
curvature(:,1) = (1+firstRatio)*curvature(:,2)-firstRatio*curvature(:,3);
curvature(:,n) = (1+lastRatio)*curvature(:,n-1)-lastRatio*curvature(:,n-2);

coefficients = zeros(size(qNodes,1),n-1,4);
for k = 1:n-1
    coefficients(:,k,1) = qNodes(:,k);
    coefficients(:,k,2) = (qNodes(:,k+1)-qNodes(:,k))/h(k) - ...
        h(k)*(2*curvature(:,k)+curvature(:,k+1))/6;
    coefficients(:,k,3) = curvature(:,k)/2;
    coefficients(:,k,4) = (curvature(:,k+1)-curvature(:,k))/(6*h(k));
end
if any(~isfinite(coefficients(:)))
    error('PathProgress:InvalidCoefficients','The cubic solve produced nonfinite coefficients.');
end
end
