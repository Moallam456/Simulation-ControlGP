function [t, s, sDot, sDDot, sDDDot, info] = ...
    sCurveTimeScaling(vMax, aMax, jMax, dt)
%SCURVETIMESCALING Symmetric jerk-limited S-curve time scaling.
%
% Inputs:
%   vMax - Maximum normalized path velocity [1/s]
%   aMax - Maximum normalized path acceleration [1/s^2]
%   jMax - Maximum normalized path jerk [1/s^3]
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
% The standard symmetric S-curve contains up to seven stages:
%
%   1. +jMax       -> acceleration rises
%   2. zero jerk   -> constant positive acceleration
%   3. -jMax       -> acceleration returns to zero
%   4. zero jerk   -> constant velocity
%   5. -jMax       -> negative acceleration develops
%   6. zero jerk   -> constant negative acceleration
%   7. +jMax       -> acceleration returns to zero
%
%
% Jerk:
%
%        +j       _____                 _____
%                |     |               |     |
%       0  ______|     |_______ _______|     |______
%                      |       |
%        -j            |_______|
%
%
% Unlike the ideal trapezoidal velocity profile, acceleration does
% not change instantaneously.
%
%
% IMPORTANT:
%   vMax, aMax, and jMax are limits on the NORMALIZED path parameter s.
%
% They are NOT directly joint limits or TCP limits.
%
% For a straight Cartesian path of length L:
%
%   TCP speed = L * sDot
%
%   TCP acceleration = L * sDDot
%
%   TCP jerk = L * sDDDot
%
%
% The function automatically reduces or removes phases when the
% normalized move from s = 0 to s = 1 is too short to reach one of
% the requested limits.


%% Validate inputs

if ~isscalar(vMax) || vMax <= 0
    error("vMax must be a positive scalar.");
end

if ~isscalar(aMax) || aMax <= 0
    error("aMax must be a positive scalar.");
end

if ~isscalar(jMax) || jMax <= 0
    error("jMax must be a positive scalar.");
end

if ~isscalar(dt) || dt <= 0
    error("dt must be a positive scalar.");
end


%% Normalized path distance

D = 1;


%% ========================================================================
%  DETERMINE WHICH LIMITS CAN ACTUALLY BE REACHED
% =========================================================================

% First determine the velocity at which aMax would just be reached
% without requiring a constant-acceleration phase.
%
% During a jerk-limited acceleration ramp:
%
%       a = jMax * tJ
%
% and the velocity where aMax is just touched is:
%
%       v = aMax^2 / jMax

vAtAMax = aMax^2 / jMax;


% Minimum total normalized distance needed to reach aMax and then
% symmetrically decelerate, with no constant-acceleration or cruise phase.

distanceToReachAMax = 2 * aMax^3 / jMax^2;


%% Case A:
% Requested maximum velocity is reached BEFORE aMax would be reached.

if vMax < vAtAMax

    % Time under positive jerk needed to build enough acceleration
    % to reach vMax in a jerk-only acceleration phase.

    tJForVMax = sqrt(vMax / jMax);


    % Distance required to:
    %
    %   accelerate to vMax
    %   then decelerate back to zero
    %
    % with jerk-only acceleration/deceleration.

    distanceForVMax = 2 * vMax * tJForVMax;


    if D >= distanceForVMax

        % ----------------------------------------------------------
        % Reach vMax, but NOT aMax.
        % Constant-velocity phase exists.
        % ----------------------------------------------------------

        vPeak = vMax;

        tJ = tJForVMax;

        % No constant-acceleration phase.
        tA = 0;

        % Actual peak acceleration is below aMax.
        aPeak = jMax * tJ;

        % Remaining distance becomes constant-velocity motion.
        tV = (D - distanceForVMax) / vPeak;


    else

        % ----------------------------------------------------------
        % Path is too short even to reach vMax.
        %
        % Motion becomes a pure four-jerk-segment profile:
        %
        %   +j, -j, -j, +j
        % ----------------------------------------------------------

        vPeak = ...
            (D * sqrt(jMax) / 2)^(2/3);

        tJ = sqrt(vPeak / jMax);

        tA = 0;
        tV = 0;

        aPeak = jMax * tJ;

    end


%% Case B:
% aMax can potentially be reached before vMax.

else

    % Time needed for acceleration to ramp from zero to aMax.

    tJ = aMax / jMax;


    % Distance required to reach the requested vMax while respecting
    % both aMax and jMax, then symmetrically decelerate.

    distanceForVMax = ...
        vMax * (tJ + vMax/aMax);


    if D >= distanceForVMax

        % ----------------------------------------------------------
        % Full seven-stage profile.
        %
        % We reach:
        %   jMax
        %   aMax
        %   vMax
        %
        % and have a constant-velocity phase.
        % ----------------------------------------------------------

        vPeak = vMax;
        aPeak = aMax;

        % Constant-acceleration duration.
        tA = vPeak/aMax - tJ;

        % Constant-velocity duration.
        tV = ...
            (D - distanceForVMax) / vPeak;


    elseif D >= distanceToReachAMax

        % ----------------------------------------------------------
        % aMax is reached, but vMax is NOT reached.
        %
        % Constant velocity disappears.
        % ----------------------------------------------------------

        aPeak = aMax;

        tV = 0;


        % Solve the distance equation for the actual peak velocity.

        vPeak = ...
            ( ...
                -aMax^2/jMax ...
                + sqrt( ...
                    (aMax^2/jMax)^2 ...
                    + 4*aMax*D) ...
            ) / 2;


        % Constant-acceleration duration.

        tA = vPeak/aMax - tJ;


    else

        % ----------------------------------------------------------
        % Path is too short to reach either:
        %
        %   aMax
        %   vMax
        %
        % Only jerk-limited ramps remain.
        % ----------------------------------------------------------

        vPeak = ...
            (D * sqrt(jMax) / 2)^(2/3);

        tJ = sqrt(vPeak / jMax);

        tA = 0;
        tV = 0;

        aPeak = jMax * tJ;

    end

end


%% ========================================================================
%  DEFINE SEVEN SEGMENT DURATIONS
% =========================================================================

% Standard symmetric S-curve:
%
% Segment 1: +j
% Segment 2:  0 jerk, +a
% Segment 3: -j
% Segment 4:  0 jerk, constant velocity
% Segment 5: -j
% Segment 6:  0 jerk, -a
% Segment 7: +j
%
% Some durations may be zero depending on the move.

segmentDuration = [
    tJ
    tA
    tJ
    tV
    tJ
    tA
    tJ
];


segmentJerk = [
     jMax
     0
    -jMax
     0
    -jMax
     0
     jMax
];


%% Total motion duration

T = sum(segmentDuration);


%% Generate time samples

t = 0:dt:T;

if t(end) < T
    t = [t T];
end


%% ========================================================================
%  PRECOMPUTE STATE AT THE START OF EVERY SEGMENT
% =========================================================================

% State variables:
%
%   s     = normalized position
%   sDot  = normalized velocity
%   sDDot = normalized acceleration
%
% For a segment with CONSTANT jerk j:
%
%   a(t) = a0 + j*t
%
%   v(t) = v0 + a0*t + 1/2*j*t^2
%
%   s(t) = s0 + v0*t + 1/2*a0*t^2 + 1/6*j*t^3


numSegments = 7;


segmentStartTime = zeros(numSegments,1);

sStart = zeros(numSegments,1);
vStart = zeros(numSegments,1);
aStart = zeros(numSegments,1);


% Initial state.
currentTime = 0;

currentS = 0;
currentV = 0;
currentA = 0;


for k = 1:numSegments

    % Store state at beginning of segment k.
    segmentStartTime(k) = currentTime;

    sStart(k) = currentS;
    vStart(k) = currentV;
    aStart(k) = currentA;


    % Duration and jerk of this segment.
    h = segmentDuration(k);
    j = segmentJerk(k);


    % Integrate constant jerk over the entire segment so that the
    % ending state becomes the starting state of the next segment.

    nextS = ...
        currentS ...
        + currentV*h ...
        + 0.5*currentA*h^2 ...
        + (1/6)*j*h^3;


    nextV = ...
        currentV ...
        + currentA*h ...
        + 0.5*j*h^2;


    nextA = ...
        currentA ...
        + j*h;


    % Update state.
    currentS = nextS;
    currentV = nextV;
    currentA = nextA;

    currentTime = currentTime + h;

end


%% Segment ending times

segmentEndTime = ...
    segmentStartTime + segmentDuration;


%% ========================================================================
%  EVALUATE TRAJECTORY AT EVERY TIME SAMPLE
% =========================================================================

s        = zeros(size(t));
sDot     = zeros(size(t));
sDDot    = zeros(size(t));
sDDDot   = zeros(size(t));


for i = 1:length(t)

    ti = t(i);


    % Find which of the seven segments contains this time.
    %
    % The tolerance prevents floating-point boundary problems.

    k = find( ...
        ti <= segmentEndTime + 1e-12, ...
        1, ...
        'first');


    % The final time may occasionally miss a boundary because of
    % floating-point precision.
    if isempty(k)
        k = numSegments;
    end


    % Time measured from the beginning of the current segment.

    tau = ti - segmentStartTime(k);


    % Current segment jerk.

    j = segmentJerk(k);


    % --------------------------------------------------------------
    % Acceleration
    % --------------------------------------------------------------

    sDDot(i) = ...
        aStart(k) ...
        + j*tau;


    % --------------------------------------------------------------
    % Velocity
    % --------------------------------------------------------------

    sDot(i) = ...
        vStart(k) ...
        + aStart(k)*tau ...
        + 0.5*j*tau^2;


    % --------------------------------------------------------------
    % Position
    % --------------------------------------------------------------

    s(i) = ...
        sStart(k) ...
        + vStart(k)*tau ...
        + 0.5*aStart(k)*tau^2 ...
        + (1/6)*j*tau^3;


    % --------------------------------------------------------------
    % Jerk
    % --------------------------------------------------------------

    sDDDot(i) = j;

end


%% Force exact endpoint conditions

s(1)   = 0;
s(end) = 1;

sDot(1)   = 0;
sDot(end) = 0;

sDDot(1)   = 0;
sDDot(end) = 0;


%% ========================================================================
%  PROFILE INFORMATION
% =========================================================================

info.type = "S-Curve";

info.T = T;
info.dt = dt;

info.requestedVMax = vMax;
info.requestedAMax = aMax;
info.requestedJMax = jMax;

info.vPeak = vPeak;
info.aPeak = aPeak;
info.jPeak = jMax;

info.tJerk = tJ;
info.tConstantAcceleration = tA;
info.tConstantVelocity = tV;

info.segmentDurations = segmentDuration;

end