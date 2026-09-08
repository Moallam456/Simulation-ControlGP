function pPath = linearInterpolation(pStart, pEnd, s)
%LINEARINTERPOLATION Generate a straight Cartesian path between two points.
%
% Inputs:
%   pStart  - Starting TCP position, 3x1 vector [m]
%   pEnd    - Ending TCP position,   3x1 vector [m]
%   s       - Path parameter values, 1xN vector where 0 <= s <= 1
%
% Output:
%   pPath   - Interpolated Cartesian positions, 3xN [m]
%
% The path is defined by:
%
%   p(s) = pStart + s * (pEnd - pStart)
%
% where s is a geometric path parameter:
%
%   s = 0  -> pStart
%   s = 1  -> pEnd


%% Validate inputs

% Both Cartesian positions must contain x, y, and z.
if numel(pStart) ~= 3 || numel(pEnd) ~= 3
    error("pStart and pEnd must each contain three values [x; y; z].");
end

% Force positions into 3x1 column-vector form.
pStart = pStart(:);
pEnd   = pEnd(:);

% Force s into a 1xN row vector.
s = s(:).';

% s must remain within the path interval [0,1].
if any(s < 0) || any(s > 1)
    error("All values of s must lie between 0 and 1.");
end


%% Linear Cartesian interpolation

% For every value of s:
%
%   p(s) = pStart + s * (pEnd - pStart)
%
% In MATLAB, (pEnd - pStart) is 3x1 and s is 1xN.
% Their product therefore produces all N interpolated 3D positions.

pPath = pStart + (pEnd - pStart) * s;

end