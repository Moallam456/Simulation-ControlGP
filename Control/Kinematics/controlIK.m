function [qSolution, info] = controlIK(robot, T_B_TCP_target, qSeed)
%CONTROLIK Numerical inverse kinematics using Damped Least Squares.
%
% Inputs:
%   robot              - Shared robot configuration
%   T_B_TCP_target     - Desired 4x4 TCP pose expressed in Base frame
%   qSeed              - Initial 6x1 joint configuration [rad]
%
% Outputs:
%   qSolution          - 6x1 joint solution [rad]
%   info               - Structure containing convergence information

%% Solver parameters

maxIterations = 1000;

positionTolerance = 1e-4;      % meters
orientationTolerance = 1e-4;   % radians

lambda = 0.01;                 % Damping factor
stepSize = 0.5;                % Joint update scaling

%% Read joint limits

qMin = robot.limits.qMin;
qMax = robot.limits.qMax;

%% Initialize solution

q = qSeed(:);

%% Validate input dimensions

if numel(q) ~= robot.dof
error("qSeed must contain one value for each robot joint.");
end

if ~isequal(size(T_B_TCP_target), [4 4])
error("T_B_TCP_target must be a 4x4 homogeneous transformation.");
end

%% Ensure initial configuration is inside joint limits

q = min(max(q, qMin), qMax);

%% Extract target pose

pTarget = T_B_TCP_target(1:3,4);

RTarget = T_B_TCP_target(1:3,1:3);

%% Initialize information structure

info.converged = false;
info.iterations = 0;
info.positionError = inf;
info.orientationError = inf;

%% Iterative IK loop

for iteration = 1:maxIterations

%--------------------------------------------------------------
% 1. Calculate current TCP pose using Control-team FK
%--------------------------------------------------------------

TCurrent = controlFK(robot, q);

pCurrent = TCurrent(1:3,4);
RCurrent = TCurrent(1:3,1:3);


%--------------------------------------------------------------
% 2. Calculate position error
%--------------------------------------------------------------

positionErrorVector = pTarget - pCurrent;

positionError = norm(positionErrorVector);


%--------------------------------------------------------------
% 3. Calculate orientation error
%
% Rotation error from current orientation to target orientation
%--------------------------------------------------------------

RError = RTarget * RCurrent';

orientationErrorVector = 0.5 * [
    RError(3,2) - RError(2,3)
    RError(1,3) - RError(3,1)
    RError(2,1) - RError(1,2)
];

orientationError = norm(orientationErrorVector);


%--------------------------------------------------------------
% 4. Check convergence
%--------------------------------------------------------------

if positionError < positionTolerance && ...
   orientationError < orientationTolerance

    info.converged = true;
    info.iterations = iteration;
    info.positionError = positionError;
    info.orientationError = orientationError;

    qSolution = q;
    return

end


%--------------------------------------------------------------
% 5. Calculate Control-team Jacobian
%--------------------------------------------------------------

J = controlJacobian(robot, q);


%--------------------------------------------------------------
% 6. Construct pose error vector
%
% Jacobian convention:
% [v; omega] = J * q_dot
%--------------------------------------------------------------

poseError = [
    positionErrorVector
    orientationErrorVector
];


%--------------------------------------------------------------
% 7. Damped Least Squares joint update
%
% dq = J' * inv(J*J' + lambda^2*I) * poseError
%
% Use matrix division instead of inv()
%--------------------------------------------------------------

dq = J' * ...
    ((J * J' + lambda^2 * eye(6)) \ poseError);


%--------------------------------------------------------------
% 8. Update joint configuration
%--------------------------------------------------------------

q = q + stepSize * dq;


%--------------------------------------------------------------
% 9. Enforce joint limits
%--------------------------------------------------------------

q = min(max(q, qMin), qMax);


%--------------------------------------------------------------
% Store latest information
%--------------------------------------------------------------

info.iterations = iteration;
info.positionError = positionError;
info.orientationError = orientationError;

end

%% If maximum iterations are reached

qSolution = q;

info.converged = false;

end
