function [waypoints, arcInfo] = generateCircleWaypoints( ...
    P_start, P_mid, P_end, numWaypoints)
%GENERATECIRCLEWAYPOINTS Generate equally spaced Cartesian waypoints
% along a circular arc defined by three points (start, mid, end).
%
% Inputs:
%   P_start      - 3x1 starting position  [x y z]
%   P_mid        - 3x1 mid-arc position   (fixes plane + sweep direction)
%   P_end        - 3x1 ending position    [x y z]
%   numWaypoints - Number of waypoints including start and end
%
% Outputs:
%   waypoints - 3xN matrix of positions on the arc
%   arcInfo   - struct with fields:
%                 .center    3x1 circle center
%                 .radius    scalar radius
%                 .axis      3x1 unit plane normal (rotation axis)
%                 .thetaTotal total signed sweep angle [rad]
%                 .e1, .e2   in-plane orthonormal basis
%                            (e1 = unit vector from center to P_start)

%% Validate inputs
if numel(P_start) ~= 3 || numel(P_mid) ~= 3 || numel(P_end) ~= 3
    error('All three points must contain 3 coordinates.');
end
if numWaypoints < 2
    error('numWaypoints must be at least 2.');
end

P_start = P_start(:);
P_mid   = P_mid(:);
P_end   = P_end(:);

%% Plane normal from the three points
v1    = P_mid - P_start;
v2    = P_end - P_start;
n_raw = cross(v1, v2);

if norm(n_raw) < 1e-12
    error('The three points are collinear; cannot define a circle.');
end

b = n_raw / norm(n_raw);

%% Circumcenter (solve linear system in 3D)
% |C - P_start| = |C - P_mid| = |C - P_end|, and C lies in the plane.
A = [ (P_mid - P_start)';
      (P_end - P_start)';
      b' ];
rhs = [ 0.5 * (P_mid'*P_mid - P_start'*P_start);
        0.5 * (P_end'*P_end - P_start'*P_start);
        b' * P_start ];
C = A \ rhs;

radius = norm(P_start - C);
if radius < 1e-9
    error('Degenerate circle: radius is essentially zero.');
end

%% In-plane orthonormal basis
e1 = (P_start - C) / radius;      % start radial direction
e2 = cross(b, e1);                % 90 deg from e1 in the plane

%% Angles of mid / end in this basis (start is at 0 by construction)
thetaMid = atan2( dot(P_mid - C, e2), dot(P_mid - C, e1) );
thetaEnd = atan2( dot(P_end - C, e2), dot(P_end - C, e1) );

%% Pick the sweep direction that actually passes through P_mid
% CCW candidate (positive sweep about b)
sweepCCW = mod(thetaEnd, 2*pi);
if sweepCCW < 1e-12
    sweepCCW = 2*pi;
end
thetaMidCCW = mod(thetaMid, 2*pi);
ccwWorks    = (thetaMidCCW <= sweepCCW + 1e-12) && (thetaMidCCW > 1e-12);

% CW candidate (negative sweep about b)
sweepCW = -mod(-thetaEnd, 2*pi);
if sweepCW > -1e-12
    sweepCW = -2*pi;
end
thetaMidCW = -mod(-thetaMid, 2*pi);
cwWorks    = (thetaMidCW >= sweepCW - 1e-12) && (thetaMidCW < -1e-12);

if ccwWorks && ~cwWorks
    thetaTotal = sweepCCW;
elseif cwWorks && ~ccwWorks
    thetaTotal = sweepCW;
else
    error('Could not determine arc direction; check the three points.');
end

%% Generate waypoints uniformly along the arc
waypoints = zeros(3, numWaypoints);

for i = 1:numWaypoints
    s     = (i - 1) / (numWaypoints - 1);
    theta = thetaTotal * s;
    waypoints(:,i) = C + radius * (cos(theta)*e1 + sin(theta)*e2);
end

%% Package arc metadata
arcInfo.center     = C;
arcInfo.radius     = radius;
arcInfo.axis       = b;
arcInfo.thetaTotal = thetaTotal;
arcInfo.e1         = e1;
arcInfo.e2         = e2;

end