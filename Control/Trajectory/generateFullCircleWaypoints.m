function [waypoints, arcInfo] = generateFullCircleWaypoints( ...
    P1, P2, P3, direction, numWaypoints)
%GENERATEFULLCIRCLEWAYPOINTS Sample a full 360-degree circle through 3 points.
%
% Inputs:
%   P1, P2, P3   - 3x1 or 1x3 positions on the circle (non-collinear)
%   direction    - +1 = CCW about plane normal cross(P2-P1, P3-P1)
%                  -1 = CW  about that same normal
%   numWaypoints - number of samples INCLUDING the closing point
%
% Outputs:
%   waypoints - 3xN, with waypoints(:,1) == waypoints(:,end) == P1
%   arcInfo   - .center, .radius, .axis, .thetaTotal (= ±2π), .e1, .e2

if numel(P1) ~= 3 || numel(P2) ~= 3 || numel(P3) ~= 3
    error('All three points must contain 3 coordinates.');
end
if numWaypoints < 3
    error('numWaypoints must be at least 3.');
end
if direction ~= 1 && direction ~= -1
    error('direction must be +1 or -1.');
end

P1 = P1(:); P2 = P2(:); P3 = P3(:);

%% Plane normal
v1 = P2 - P1;
v2 = P3 - P1;
n_raw = cross(v1, v2);
if norm(n_raw) < 1e-12
    error('The three points are collinear; cannot define a circle.');
end
b = n_raw / norm(n_raw);

%% Circumcenter
A = [ (P2 - P1)';
      (P3 - P1)';
      b' ];
rhs = [ 0.5 * (P2'*P2 - P1'*P1);
        0.5 * (P3'*P3 - P1'*P1);
        b' * P1 ];
C = A \ rhs;

radius = norm(P1 - C);
if radius < 1e-9
    error('Degenerate circle: radius is essentially zero.');
end

%% In-plane orthonormal basis (start at P1)
e1 = (P1 - C) / radius;
e2 = cross(b, e1);

%% Full 2π sweep
thetaTotal = direction * 2 * pi;

%% Sample uniformly in angle (== uniformly in arc length on a circle)
waypoints = zeros(3, numWaypoints);
for i = 1:numWaypoints
    s     = (i - 1) / (numWaypoints - 1);
    theta = thetaTotal * s;
    waypoints(:,i) = C + radius * (cos(theta)*e1 + sin(theta)*e2);
end

% Kill floating-point drift so the loop is truly closed
waypoints(:,end) = P1;

arcInfo.center     = C;
arcInfo.radius     = radius;
arcInfo.axis       = b;
arcInfo.thetaTotal = thetaTotal;
arcInfo.e1         = e1;
arcInfo.e2         = e2;

end