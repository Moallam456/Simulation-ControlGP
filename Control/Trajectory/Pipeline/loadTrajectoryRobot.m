function [robot, tcpFK, solveSample] = loadTrajectoryRobot()
%LOADTRAJECTORYROBOT Bind the robot_ver2 model to column-vector trajectory data.

repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
originalPath = path;
restorePath = onCleanup(@() path(originalPath));
addpath(fullfile(repoRoot,'Final_Architecture'),'-begin');
robot = loadRobot("robot_ver2");

% The URDF rigidBodyTree uses rows and already includes the fixed flange-to-TCP
% transform. Poses are base-to-TCP transforms in metres and angles are radians.
tcpFK = @(q) getTransform(robot.model,q(:).',char(robot.structure.frames.endEffector));
solveSample = @(target,previous) solveNearestBranch(robot,target,previous);
end

function [q,info] = solveNearestBranch(robot,target,previous)
previous = previous(:).';
limits = robot.params.joints.positionLimits;
if numel(previous) ~= robot.structure.dof || any(~isfinite(previous)) || ...
        any(previous < limits(:,1).' | previous > limits(:,2).')
    error('TrajectoryIK:InvalidSeed','The preceding joints must be finite and within robot_ver2 limits.');
end

% Preserve the solver's FK tolerances (1e-5 m and 1e-4 rad). Its seed ordering
% uses wrapped distance; select with actual changes in the commanded angles.
solutions = robot.solveIK(target,struct('qSeed',previous));
info = solutions.info;
q = [];
if ~solutions.valid
    return;
end

candidates = solutions.q;
turn = 2*pi;
minimumTurn = ceil((limits(:,1).'-candidates)/turn);
maximumTurn = floor((limits(:,2).'-candidates)/turn);
nearestTurn = round((previous-candidates)/turn);
candidates = candidates + turn*min(max(nearestTurn,minimumTurn),maximumTurn);
delta = candidates - previous;
valid = all(isfinite(candidates) & candidates >= limits(:,1).' & ...
    candidates <= limits(:,2).',2);

% Bound the raw step at the demos' 0.05 s sampling. This rejects branch
% jumps, including jumps across +/-pi, without assuming an actuator rating.
maxStep = 0.10;   % rad per sample
continuous = valid & all(abs(delta) <= maxStep,2);
if ~any(continuous)
    info.success = false;
    info.message = sprintf('No limit-valid branch within %.2f rad per joint of the preceding sample.',maxStep);
    return;
end

rows = find(continuous);
[~,closest] = min(vecnorm(delta(rows,:),2,2));
row = rows(closest);
q = candidates(row,:).';
info.positionError = info.positionErrors(row);
info.orientationError = info.orientationErrors(row);
end
