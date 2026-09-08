function [t, s, sDot, sDDot, sDDDot, info] = ...
    trapezoidalTimeScaling(vMax, aMax, dt)
%TRAPEZOIDALTIMESCALING Trapezoidal/triangular velocity time scaling.
%
% Inputs:
%   vMax - Maximum normalized path velocity [1/s]
%   aMax - Maximum normalized path acceleration [1/s^2]
%   dt   - Sampling period [s]
%
% Outputs:
%   t        - Time samples [s]
%   s        - Normalized path position
%   sDot     - Normalized path velocity [1/s]
%   sDDot    - Normalized path acceleration [1/s^2]
%   sDDDot   - Normalized path jerk [1/s^3]
%   info     - Information about the generated profile
%
%
% The normalized path has total distance:
%
%   s : 0 -> 1
%
%
% TRAPEZOIDAL PROFILE
%
%   Phase 1: accelerate at +a
%   Phase 2: move at constant velocity
%   Phase 3: decelerate at -a
%
%
% Velocity shape:
%
%             ___________
%            /           \
%           /             \
%   _______/               \_______
%
%
% TRIANGULAR CASE
%
% If there is not enough normalized path distance to reach vMax,
% the constant-velocity phase disappears:
%
%             /\
%            /  \
%   ________/    \________
%
%
% IMPORTANT:
%   vMax and aMax refer to the normalized path parameter s.
%
% They are NOT directly:
%   - TCP velocity [m/s]
%   - joint velocity [rad/s]
%
% For a straight Cartesian path of length L:
%
%   TCP speed        = L * sDot
%   TCP acceleration = L * sDDot
%
%
% Tunable parameters:
%   vMax
%   aMax
%   dt


%% Validate inputs

if ~isscalar(vMax) || vMax <= 0
    error("vMax must be a positive scalar.");
end

if ~isscalar(aMax) || aMax <= 0
    error("aMax must be a positive scalar.");
end

if ~isscalar(dt) || dt <= 0
    error("dt must be a positive scalar.");
end


%% Normalized path length

% s always travels from 0 to 1.

D = 1;


%% Determine whether the profile is trapezoidal or triangular

% To accelerate from zero to vMax:
%
%   tA = vMax/aMax
%
% Distance used during acceleration and deceleration together:
%
%   D_required = vMax^2 / aMax
%
% If:
%
%   vMax^2/aMax <= 1
%
% there is enough distance to reach vMax.
%
% Otherwise, the robot must start decelerating before reaching vMax.

if (vMax^2 / aMax) <= D

    % --------------------------------------------------------------
    % TRUE TRAPEZOIDAL PROFILE
    % --------------------------------------------------------------

    isTriangular = false;

    % Requested maximum velocity can be reached.
    vPeak = vMax;

    % Acceleration duration.
    tAccel = vPeak / aMax;

    % Distance used during acceleration.
    sAccel = 0.5 * aMax * tAccel^2;

    % Remaining distance is travelled at constant velocity.
    sCruise = D - 2*sAccel;

    % Duration of constant-velocity phase.
    tCruise = sCruise / vPeak;

else

    % --------------------------------------------------------------
    % TRIANGULAR PROFILE
    % --------------------------------------------------------------

    isTriangular = true;

    % No constant-velocity phase exists.
    tCruise = 0;

    % For a triangular profile:
    %
    %   D = vPeak^2 / aMax
    %
    % therefore:

    vPeak = sqrt(aMax * D);

    % Time needed to reach this peak velocity.
    tAccel = vPeak / aMax;

end


%% Total trajectory duration

T = 2*tAccel + tCruise;


%% Generate time samples

t = 0:dt:T;

if t(end) < T
    t = [t T];
end


%% Allocate outputs

s        = zeros(size(t));
sDot     = zeros(size(t));
sDDot    = zeros(size(t));
sDDDot   = zeros(size(t));


%% Useful switching times

t1 = tAccel;
t2 = tAccel + tCruise;


%% Position at end of acceleration

s1 = 0.5 * aMax * tAccel^2;


%% Position at end of cruise

s2 = s1 + vPeak*tCruise;


%% Evaluate each time sample

for i = 1:length(t)

    ti = t(i);


    if ti <= t1

        % ==========================================================
        % PHASE 1: CONSTANT POSITIVE ACCELERATION
        % ==========================================================

        sDDot(i) = aMax;

        sDot(i) = aMax * ti;

        s(i) = 0.5 * aMax * ti^2;


    elseif ti <= t2

        % ==========================================================
        % PHASE 2: CONSTANT VELOCITY
        % ==========================================================

        sDDot(i) = 0;

        sDot(i) = vPeak;

        % Time spent inside the cruise phase.
        tau = ti - t1;

        s(i) = s1 + vPeak*tau;


    else

        % ==========================================================
        % PHASE 3: CONSTANT NEGATIVE ACCELERATION
        % ==========================================================

        % Time since deceleration started.
        tau = ti - t2;

        sDDot(i) = -aMax;

        sDot(i) = vPeak - aMax*tau;

        s(i) = ...
            s2 ...
            + vPeak*tau ...
            - 0.5*aMax*tau^2;

    end

end


%% Force exact final state

s(1)       = 0;
s(end)     = 1;

sDot(1)    = 0;
sDot(end)  = 0;


%% Jerk handling

% Within every trapezoidal phase, acceleration is constant.
%
% Therefore:
%
%   jerk = 0
%
% inside the phases.
%
% But acceleration changes instantaneously at the switching points.
% The ideal mathematical jerk there is not finite.
%
% NaN is used at the nearest sampled switching points to indicate
% that these points should NOT be interpreted as zero jerk.

switchTimes = [0, t1, t2, T];

for k = 1:length(switchTimes)

    [~,idx] = min(abs(t - switchTimes(k)));

    sDDDot(idx) = NaN;

end


%% Profile information

info.type = "Trapezoidal";

if isTriangular
    info.type = "Triangular";
end

info.T = T;
info.dt = dt;

info.requestedVMax = vMax;
info.aMax = aMax;

info.vPeak = vPeak;

info.tAccel = tAccel;
info.tCruise = tCruise;

info.isTriangular = isTriangular;

end