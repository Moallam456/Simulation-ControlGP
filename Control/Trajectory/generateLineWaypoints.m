function waypoints = generateLineWaypoints(P_start, P_end, numWaypoints)
%GENERATELINEWAYPOINTS Generate equally spaced Cartesian waypoints
% along a straight line between two points.
%
% Inputs:
%   P_start      - 3x1 or 1x3 starting position [x y z]
%   P_end        - 3x1 or 1x3 ending position [x y z]
%   numWaypoints - Number of waypoints including start and end
%
% Output:
%   waypoints    - 3xN matrix containing the generated positions

%% Validate inputs

if numel(P_start) ~= 3
    error("P_start must contain 3 coordinates.");
end

if numel(P_end) ~= 3
    error("P_end must contain 3 coordinates.");
end

if numWaypoints < 2
    error("numWaypoints must be at least 2.");
end

%% Convert points to column vectors

P_start = P_start(:);
P_end = P_end(:);

%% Generate straight-line waypoints

waypoints = zeros(3, numWaypoints);

for i = 1:numWaypoints

    % Normalized position along the line
    s = (i - 1) / (numWaypoints - 1);

    % Linear interpolation
    waypoints(:,i) = P_start + s * (P_end - P_start);

end

end