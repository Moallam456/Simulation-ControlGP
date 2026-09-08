function [TPath, sValues] = generateCartesianPosePath( ...
    pStart, pEnd, RStart, REnd, numWaypoints)
%GENERATECARTESIANPOSEPATH Generate Cartesian TCP poses between two poses.
%
% Position is interpolated linearly.
% Orientation is interpolated using SLERP.
%
% Inputs:
%   pStart       - Starting TCP position, 3x1 [m]
%   pEnd         - Ending TCP position,   3x1 [m]
%   RStart       - Starting orientation, 3x3 rotation matrix
%   REnd         - Ending orientation,   3x3 rotation matrix
%   numWaypoints - Number of poses including start and end
%
% Outputs:
%   TPath        - 4x4xN array of homogeneous TCP poses
%   sValues      - 1xN normalized path parameters from 0 to 1
%
% For constant orientation:
%
%   REnd = RStart;
%
% Each pose has the form:
%
%       T = [ R  p
%             0  1 ]

%% Validate inputs

if numel(pStart) ~= 3
    error("pStart must contain 3 coordinates.");
end

if numel(pEnd) ~= 3
    error("pEnd must contain 3 coordinates.");
end

if ~isequal(size(RStart), [3 3])
    error("RStart must be a 3x3 rotation matrix.");
end

if ~isequal(size(REnd), [3 3])
    error("REnd must be a 3x3 rotation matrix.");
end

if numWaypoints < 2 || fix(numWaypoints) ~= numWaypoints
    error("numWaypoints must be an integer greater than or equal to 2.");
end

%% Convert positions to column vectors

pStart = pStart(:);
pEnd   = pEnd(:);

%% Generate normalized path parameter

sValues = linspace(0, 1, numWaypoints);

%% Preallocate pose path

TPath = zeros(4, 4, numWaypoints);

%% Generate each Cartesian pose

for i = 1:numWaypoints

    s = sValues(i);

    % ---------------------------------------------------------
    % Position interpolation
    % ---------------------------------------------------------

    p = linearInterpolation( ...
        pStart, ...
        pEnd, ...
        s);

    % ---------------------------------------------------------
    % Orientation interpolation
    % ---------------------------------------------------------

    R = slerpOrientation( ...
        RStart, ...
        REnd, ...
        s);

    % ---------------------------------------------------------
    % Assemble homogeneous transformation
    % ---------------------------------------------------------

    T = eye(4);

    T(1:3,1:3) = R;
    T(1:3,4)   = p;

    TPath(:,:,i) = T;

end

end