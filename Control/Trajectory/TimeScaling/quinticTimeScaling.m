function [t, s, sDot, sDDot, sDDDot, info] = quinticTimeScaling(T, dt)
%QUINTICTIMESCALING Fifth-order polynomial time scaling.
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
% Quintic scaling satisfies six endpoint conditions:
%
%   s(0)     = 0
%   s(T)     = 1
%
%   sDot(0)  = 0
%   sDot(T)  = 0
%
%   sDDot(0) = 0
%   sDDot(T) = 0
%
% The normalized equation is:
%
%   s(t) = 10(t/T)^3
%        - 15(t/T)^4
%        +  6(t/T)^5
%
% Compared with cubic scaling, quintic scaling also forces the
% acceleration to zero at the beginning and end.
%
% Tunable parameters:
%   T  - total trajectory duration
%   dt - trajectory sampling period


%% Validate inputs

if ~isscalar(T) || T <= 0
    error("T must be a positive scalar.");
end

if ~isscalar(dt) || dt <= 0
    error("dt must be a positive scalar.");
end


%% Generate time vector

t = 0:dt:T;

if t(end) < T
    t = [t T];
end


%% Normalize time

tau = t / T;


%% Quintic path parameter

s = ...
    10*tau.^3 ...
    - 15*tau.^4 ...
    + 6*tau.^5;


%% First derivative: path velocity

sDot = ...
    (30*tau.^2 ...
    - 60*tau.^3 ...
    + 30*tau.^4) / T;


%% Second derivative: path acceleration

sDDot = ...
    (60*tau ...
    - 180*tau.^2 ...
    + 120*tau.^3) / T^2;


%% Third derivative: path jerk

sDDDot = ...
    (60 ...
    - 360*tau ...
    + 360*tau.^2) / T^3;


%% Force exact endpoint values

s(1)   = 0;
s(end) = 1;

sDot(1)   = 0;
sDot(end) = 0;

sDDot(1)   = 0;
sDDot(end) = 0;


%% Profile information

info.type = "Quintic";
info.T = T;
info.dt = dt;

info.peakPathVelocity = max(abs(sDot));
info.peakPathAcceleration = max(abs(sDDot));
info.peakPathJerk = max(abs(sDDDot));

end