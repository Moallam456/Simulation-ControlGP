function RInterp = slerpOrientation(RStart, REnd, s)
%SLERPORIENTATION Interpolate between two orientations using SLERP.
%
% Inputs:
%   RStart  - Starting rotation matrix, 3x3
%   REnd    - Ending rotation matrix,   3x3
%   s       - Normalized interpolation parameter
%
%             s = 0  -> starting orientation
%             s = 1  -> ending orientation
%
% Output:
%   RInterp - Interpolated 3x3 rotation matrix
%
% The interpolation is performed using unit quaternions.
%
% Quaternion convention used by MATLAB:
%
%       q = [w x y z]
%
% where:
%   w = scalar component
%   x,y,z = vector components


%% Validate inputs

if ~isequal(size(RStart), [3 3])
    error("RStart must be a 3x3 rotation matrix.");
end

if ~isequal(size(REnd), [3 3])
    error("REnd must be a 3x3 rotation matrix.");
end

if ~isscalar(s) || s < 0 || s > 1
    error("s must be a scalar between 0 and 1.");
end


%% Convert rotation matrices to quaternions
%
% MATLAB rotm2quat returns:
%
%       [w x y z]

qStart = rotm2quat(RStart);
qEnd   = rotm2quat(REnd);


%% Ensure shortest rotational path
%
% A quaternion q and -q represent exactly the same orientation.
%
% However, when interpolating, they correspond to different paths
% around the unit quaternion sphere.
%
% If the dot product is negative, negate qEnd so SLERP follows
% the shorter rotation.

dotProduct = dot(qStart, qEnd);

if dotProduct < 0

    qEnd = -qEnd;

    dotProduct = -dotProduct;

end


%% Protect against numerical roundoff
%
% acos() requires its input to remain inside [-1,1].

dotProduct = max(-1, min(1, dotProduct));


%% Handle orientations that are extremely close
%
% When theta is very small:
%
%       sin(theta) ~= 0
%
% and the standard SLERP equation becomes numerically sensitive.
%
% In this case, normalized linear interpolation gives practically
% the same result.

if dotProduct > 0.9995

    qInterp = ...
        (1 - s) * qStart + ...
        s * qEnd;

    % Restore unit quaternion length.
    qInterp = qInterp / norm(qInterp);

else

    %% Standard SLERP
    %
    % Angle between the two unit quaternions.

    theta = acos(dotProduct);

    sinTheta = sin(theta);


    % SLERP equation:
    %
    % q(s) =
    %
    % sin((1-s)*theta)
    % ---------------- * qStart
    %    sin(theta)
    %
    %        +
    %
    % sin(s*theta)
    % ------------ * qEnd
    %  sin(theta)

    weightStart = ...
        sin((1 - s) * theta) / sinTheta;

    weightEnd = ...
        sin(s * theta) / sinTheta;


    qInterp = ...
        weightStart * qStart + ...
        weightEnd * qEnd;


    % Numerical safety: ensure result remains a unit quaternion.
    qInterp = qInterp / norm(qInterp);

end


%% Convert interpolated quaternion back to rotation matrix

RInterp = quat2rotm(qInterp);

end