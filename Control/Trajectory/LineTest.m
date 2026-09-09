%% Line Path + Arc-Length Parameterization + IK Feasibility Test

clear;
clc;
close all;

%% Robot
robot = config.UR5();

%% User-defined Cartesian points
% Each column is one point [x; y; z]

P = [
    -0.8173,  -0.7673,  -0.7173;
    -0.1915,  -0.1415,  -0.0915;
    -0.0055,  -0.0055,  -0.0055
];

%% Path generation parameters
numPointsPerSegment = 100;

%% Arc-length parameterization
arcLengthSpacing = 0.005;     % 5 mm

%% Initial IK seed
qSeed = zeros(robot.dof,1);

%% Fixed orientation
T_start = controlFK(robot,qSeed);
R_target = T_start(1:3,1:3);

%% ---------------------------------------------------------
% 1. Generate geometric path
% ----------------------------------------------------------

pathPoints = [];

for i = 1:size(P,2)-1

    segment = generateLineWaypoints( ...
        P(:,i), ...
        P(:,i+1), ...
        numPointsPerSegment);

    % Avoid duplicating connecting points
    if i > 1
        segment = segment(:,2:end);
    end

    pathPoints = [pathPoints, segment];

end

%% ---------------------------------------------------------
% 2. Arc-length parameterization
% ----------------------------------------------------------

[waypoints, sOriginal, sNew] = ...
    arcLengthParameterize(pathPoints,arcLengthSpacing);

numWaypoints = size(waypoints,2);

fprintf('\n');
fprintf('============================================\n');
fprintf(' Path Generation + Arc-Length Parameterization\n');
fprintf('============================================\n');

fprintf('Original path points : %d\n',size(pathPoints,2));
fprintf('Arc-length waypoints : %d\n',numWaypoints);
fprintf('Total path length    : %.4f m\n',sOriginal(end));
fprintf('Requested spacing    : %.4f m\n',arcLengthSpacing);
fprintf('--------------------------------------------\n');

%% ---------------------------------------------------------
% 3. Storage for IK results
% ----------------------------------------------------------

qWaypoints = zeros(robot.dof,numWaypoints);

fkPositions = zeros(3,numWaypoints);

positionErrors = zeros(1,numWaypoints);
orientationErrors = zeros(1,numWaypoints);

converged = false(1,numWaypoints);

%% ---------------------------------------------------------
% 4. Test IK feasibility at every waypoint
% ----------------------------------------------------------

fprintf('\n');
fprintf('Testing IK feasibility...\n\n');

for i = 1:numWaypoints

    %% Construct target pose

    T_target = eye(4);

    T_target(1:3,1:3) = R_target;
    T_target(1:3,4) = waypoints(:,i);

    %% Solve IK

    [qSolution,info] = ...
        controlIK(robot,T_target,qSeed);

    %% Store results

    qWaypoints(:,i) = qSolution;

    positionErrors(i) = info.positionError;
    orientationErrors(i) = info.orientationError;

    converged(i) = info.converged;

    %% Forward kinematics check

    T_fk = controlFK(robot,qSolution);

    fkPositions(:,i) = T_fk(1:3,4);

    %% Display result

    fprintf(['Waypoint %3d/%3d | ' ...
             'IK: %d | ' ...
             'Position Error: %.3e m | ' ...
             'Orientation Error: %.3e rad\n'], ...
             i, ...
             numWaypoints, ...
             info.converged, ...
             info.positionError, ...
             info.orientationError);

    %% Stop if IK fails

    if ~info.converged

        warning("IK failed at waypoint %d.",i);

        break;
    end

    %% Use current solution as seed for next waypoint

    qSeed = qSolution;

end

%% ---------------------------------------------------------
% 5. Feasibility summary
% ----------------------------------------------------------

numSuccessful = sum(converged);

successRate = 100 * numSuccessful / numWaypoints;

fprintf('\n');
fprintf('============================================\n');
fprintf(' IK FEASIBILITY SUMMARY\n');
fprintf('============================================\n');

fprintf('Total waypoints : %d\n',numWaypoints);
fprintf('Successful      : %d\n',numSuccessful);
fprintf('Failed          : %d\n',numWaypoints-numSuccessful);

fprintf('Success rate    : %.2f %%\n',successRate);

fprintf('Maximum position error    : %.3e m\n', ...
        max(positionErrors));

fprintf('Maximum orientation error : %.3e rad\n', ...
        max(orientationErrors));

%% ---------------------------------------------------------
% 6. Plot Cartesian path
% ----------------------------------------------------------

figure;

plot3( ...
    pathPoints(1,:), ...
    pathPoints(2,:), ...
    pathPoints(3,:), ...
    'k--');

hold on;

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'bo-');

plot3( ...
    P(1,:), ...
    P(2,:), ...
    P(3,:), ...
    'rx', ...
    'MarkerSize',10, ...
    'LineWidth',2);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

legend( ...
    'Generated line path', ...
    'Arc-length waypoints', ...
    'User-defined points');

title('Cartesian Path with Arc-Length Parameterization');

%% ---------------------------------------------------------
% 7. Plot IK + FK result
% ----------------------------------------------------------

figure;

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'bo-');

hold on;

plot3( ...
    fkPositions(1,:), ...
    fkPositions(2,:), ...
    fkPositions(3,:), ...
    'rx--');

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

legend( ...
    'Desired Cartesian path', ...
    'FK from IK solutions');

title('IK Feasibility: Desired vs FK');

%% ---------------------------------------------------------
% 8. Plot IK errors
% ----------------------------------------------------------

figure;

plot(sNew,positionErrors,'o-');

grid on;

xlabel('Arc Length [m]');
ylabel('Position Error [m]');

title('IK Position Error Along Path');

figure;

plot(sNew,orientationErrors,'o-');

grid on;

xlabel('Arc Length [m]');
ylabel('Orientation Error [rad]');

title('IK Orientation Error Along Path');