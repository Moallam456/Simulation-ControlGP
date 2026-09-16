function solutions = inverseKinematics(targetPose,robot,options)

% INVERSEKINEMATICS Robot-specific IK interface for initial_trial.
%
% The current implementation uses damped least-squares numerical IK because
% the present model is a compound fixed-transform CAD-style chain. The
% earlier analytical MDH IK must be re-derived before it can be trusted for
% this exact geometry.

if nargin < 3 || isempty(options)
    options = struct();
end

if isfield(options,'qSeed') && ~isempty(options.qSeed)
    q = options.qSeed(:).';
else
    q = robot.params.joints.homePosition;
end

validateTargetPose(targetPose);

qMin = robot.params.joints.positionLimits(:,1).';
qMax = robot.params.joints.positionLimits(:,2).';
q = min(max(q,qMin),qMax);

maxIterations = getOption(options,'maxIterations',200);
positionTolerance = getOption(options,'positionTolerance',1e-6);
orientationTolerance = getOption(options,'orientationTolerance',1e-6);
lambda = getOption(options,'damping',1e-3);

success = false;
lastPositionError = Inf;
lastOrientationError = Inf;

for iteration = 1:maxIterations

    TCurrent = forward_kinematics(robot,q);
    [e,positionError,orientationError] = poseError(targetPose,TCurrent);

    lastPositionError = positionError;
    lastOrientationError = orientationError;

    if positionError < positionTolerance && orientationError < orientationTolerance
        success = true;
        break;
    end

    J = numericalJacobian(robot,q,TCurrent);
    dq = (J'/(J*J' + lambda^2*eye(6))) * e;

    stepLimit = 0.25;
    stepNorm = norm(dq);

    if stepNorm > stepLimit
        dq = dq * stepLimit/stepNorm;
    end

    q = min(max(q + dq.',qMin),qMax);

end

if success
    solutions.q = q;
    solutions.valid = true;
else
    solutions.q = zeros(0,robot.structure.dof);
    solutions.valid = false;
end

solutions.branch = strings(size(solutions.q,1),1);
solutions.info.success = success;
solutions.info.numSolutions = size(solutions.q,1);
solutions.info.iterations = iteration;
solutions.info.positionError = lastPositionError;
solutions.info.orientationError = lastOrientationError;
solutions.info.method = "damped_least_squares";

if success
    solutions.info.message = "IK converged.";
else
    solutions.info.message = "IK did not converge from the supplied seed.";
end

end

function validateTargetPose(targetPose)

if ~isequal(size(targetPose),[4 4]) || any(~isfinite(targetPose(:)))
    error('initial_trial:InvalidTargetPose', ...
        'targetPose must be a finite 4-by-4 transform.');
end

if norm(targetPose(4,:) - [0 0 0 1]) > 1e-10
    error('initial_trial:InvalidTargetPose', ...
        'targetPose last row must be [0 0 0 1].');
end

end

function value = getOption(options,name,defaultValue)

if isfield(options,name) && ~isempty(options.(name))
    value = options.(name);
else
    value = defaultValue;
end

end

function J = numericalJacobian(robot,q,TCurrent)

n = robot.structure.dof;
J = zeros(6,n);
h = 1e-6;

for i = 1:n
    qPerturbed = q;
    qPerturbed(i) = qPerturbed(i) + h;

    TPerturbed = forward_kinematics(robot,qPerturbed);
    J(:,i) = poseError(TPerturbed,TCurrent) / h;
end

end

function [e,positionError,orientationError] = poseError(TTarget,TCurrent)

positionVector = TTarget(1:3,4) - TCurrent(1:3,4);

RTarget = TTarget(1:3,1:3);
RCurrent = TCurrent(1:3,1:3);

orientationVector = 0.5 * ( ...
    cross(RCurrent(:,1),RTarget(:,1)) + ...
    cross(RCurrent(:,2),RTarget(:,2)) + ...
    cross(RCurrent(:,3),RTarget(:,3)) );

e = [positionVector; orientationVector];
positionError = norm(positionVector);
orientationError = norm(orientationVector);

end
