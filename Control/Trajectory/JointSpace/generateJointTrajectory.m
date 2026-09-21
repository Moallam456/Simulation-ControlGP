function [t, q, qDot, qDDot, qDDDot, info] = ...
    generateJointTrajectory(qStart, qGoal, T, dt)
%GENERATEJOINTTRAJECTORY Generate a synchronized joint-space trajectory.
%
% Inputs:
%   qStart  - Initial joint configuration [rad], 6x1
%   qGoal   - Final joint configuration [rad], 6x1
%   T       - Total motion duration [s]
%   dt      - Sampling period [s]
%
% Outputs:
%   t       - Time samples [s], 1xN
%   q       - Joint positions [rad], Nx6
%   qDot    - Joint velocities [rad/s], Nx6
%   qDDot   - Joint accelerations [rad/s^2], Nx6
%   qDDDot  - Joint jerk [rad/s^3], Nx6
%   info    - Information about the generated trajectory


%% ========================================================================
%  1. VALIDATE INPUTS
% =========================================================================

qStart = qStart(:);
qGoal  = qGoal(:);

if length(qStart) ~= 6 || length(qGoal) ~= 6
    error("qStart and qGoal must contain exactly 6 joint values.");
end

if T <= 0
    error("T must be positive.");
end

if dt <= 0
    error("dt must be positive.");
end


%% ========================================================================
%  2. JOINT DISPLACEMENT
% =========================================================================

deltaQ = qGoal - qStart;


%% ========================================================================
%  3. GENERATE QUINTIC TIME SCALING
% =========================================================================

[t, s, sDot, sDDot, sDDDot] = ...
    quinticTimeScaling(T, dt);


%% ========================================================================
%  4. GENERATE JOINT TRAJECTORY
% =========================================================================

q = ...
    qStart.' + ...
    s.' * deltaQ.';

qDot = ...
    sDot.' * deltaQ.';

qDDot = ...
    sDDot.' * deltaQ.';

qDDDot = ...
    sDDDot.' * deltaQ.';


%% ========================================================================
%  5. FORCE EXACT ENDPOINTS
% =========================================================================

q(1, :)   = qStart.';
q(end, :) = qGoal.';

qDot(1, :)   = 0;
qDot(end, :) = 0;

qDDot(1, :)   = 0;
qDDot(end, :) = 0;


%% ========================================================================
%  6. TRAJECTORY INFORMATION
% =========================================================================

info.type = "Joint-Space Quintic";
info.duration = T;
info.dt = dt;
info.samples = length(t);

info.qStart = qStart;
info.qGoal = qGoal;
info.deltaQ = deltaQ;

info.peakJointVelocity = ...
    max(abs(qDot), [], 1).';

info.peakJointAcceleration = ...
    max(abs(qDDot), [], 1).';


end