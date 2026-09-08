function [t, s, sDot, sDDot, sDDDot, info] = cubicTimeScaling(T, dt)
%CUBICTIMESCALING Third-order polynomial time scaling.
%
% Inputs:
%   T   - Total motion duration [s]
%   dt  - Sampling period [s]
%
% Outputs:
%   t        - Time samples [s]
%   s        - Normalized path position
%   sDot     - Normalized path velocity [1/s]
%   sDDot    - Normalized path acceleration [1/s^2]
%   sDDDot   - Normalized path jerk [1/s^3]
%   info     - Information about the generated profile
%
% The path parameter satisfies:
%
%   s(0) = 0
%   s(T) = 1
%
% and the robot starts and ends at rest:
%
%   sDot(0) = 0
%   sDot(T) = 0
%
% The cubic time-scaling equation is:
%
%   s(t) = 3(t/T)^2 - 2(t/T)^3
%
% IMPORTANT:
%   s is NOT Cartesian position and it is NOT a joint angle.
%   It is the normalized progress along an already-defined path.
%
% For example:
%
%   s = 0   -> beginning of path
%   s = 0.5 -> halfway along path
%   s = 1   -> end of path
%
% Tunable parameters:
%   T  - increasing T makes the motion slower
%   dt - smaller dt gives more trajectory samples
%
% Limitation:
%   Cubic scaling has non-zero acceleration at t = 0 and t = T.
%   If the robot is stationary before and after the trajectory,
%   acceleration therefore changes instantaneously at the boundaries.


%% Validate inputs

if ~isscalar(T) || T <= 0
    error("T must be a positive scalar.");
end

if ~isscalar(dt) || dt <= 0
    error("dt must be a positive scalar.");
end


%% Generate time vector

% Generate samples from 0 to T.
t = 0:dt:T;

% If dt does not divide T exactly, explicitly add the final sample T.
if t(end) < T
    t = [t T];
end


%% Normalize time

% tau varies from:
%
%   tau = 0 -> t = 0
%   tau = 1 -> t = T
%
% Using normalized time makes the equations easier to read.

tau = t / T;


%% Cubic path parameter

s = 3*tau.^2 - 2*tau.^3;


%% First derivative: path velocity

sDot = (6*tau - 6*tau.^2) / T;


%% Second derivative: path acceleration

sDDot = (6 - 12*tau) / T^2;


%% Third derivative: path jerk

sDDDot = (-12 / T^3) * ones(size(t));


% Inside the cubic polynomial the jerk is finite and constant.
%
% However, because acceleration at the endpoints is non-zero,
% connecting this trajectory to a stationary state causes an
% instantaneous acceleration change.
%
% Mark those boundary jerk values as undefined for comparison plots.

sDDDot(1)   = NaN;
sDDDot(end) = NaN;


%% Force exact numerical endpoint values

s(1)       = 0;
s(end)     = 1;

sDot(1)    = 0;
sDot(end)  = 0;


%% Profile information

info.type = "Cubic";
info.T = T;
info.dt = dt;

info.peakPathVelocity = max(abs(sDot));
info.peakPathAcceleration = max(abs(sDDot));

end