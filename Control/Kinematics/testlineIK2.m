%% Straight-Line Cartesian Path + IK Test

clear;
clc;
close all;

robot = config.UR5();

%% User-defined Cartesian waypoints
%
% Each column represents one Cartesian point [x; y; z] in meters.
%
% Example: three points forming two straight-line segments.

P = [
    -0.8173,  -0.7173;
    -0.1915,  -0.0915;
    -0.088,  -0.0055
];

numPointsPerSegment = 20;

%% Fixed TCP orientation

% Use the orientation of the first Cartesian point.
%
% For now, orientation is kept constant throughout the path.

qSeed = zeros(robot.dof,1);

T_start = controlFK(robot,qSeed);

R_target = T_start(1:3,1:3);

%% Generate Cartesian path

numInputPoints = size(P,2);

waypoints = [];

for i = 1:numInputPoints-1

    P_start = P(:,i);
    P_end   = P(:,i+1);

    segment = generateLineWaypoints( ...
        P_start, ...
        P_end, ...
        numPointsPerSegment);

    % Avoid duplicating the connecting point
    if i > 1
        segment = segment(:,2:end);
    end

    waypoints = [waypoints, segment];

end

numWaypoints = size(waypoints,2);

%% Storage

qWaypoints = zeros(robot.dof,numWaypoints);

fkPositions = zeros(3,numWaypoints);

positionErrors = zeros(1,numWaypoints);

orientationErrors = zeros(1,numWaypoints);

converged = false(1,numWaypoints);

%% Solve IK for every waypoint

for i = 1:numWaypoints

    %--------------------------------------------------------------
    % Create target transformation
    %--------------------------------------------------------------

    T_target = eye(4);

    T_target(1:3,1:3) = R_target;
    T_target(1:3,4) = waypoints(:,i);

    %--------------------------------------------------------------
    % Solve IK
    %--------------------------------------------------------------

    [qSolution,info] = controlIK( ...
        robot, ...
        T_target, ...
        qSeed);

    %--------------------------------------------------------------
    % Store results
    %--------------------------------------------------------------

    qWaypoints(:,i) = qSolution;

    positionErrors(i) = info.positionError;

    orientationErrors(i) = info.orientationError;

    converged(i) = info.converged;

    %--------------------------------------------------------------
    % Verify solution using FK
    %--------------------------------------------------------------

    T_fk = controlFK(robot,qSolution);

    fkPositions(:,i) = T_fk(1:3,4);

    %--------------------------------------------------------------
    % Print result
    %--------------------------------------------------------------

    fprintf(['Waypoint %3d/%3d | Converged: %d | ' ...
             'Position Error: %.3e m | ' ...
             'Orientation Error: %.3e rad | ' ...
             'Iterations: %d\n'], ...
             i, ...
             numWaypoints, ...
             info.converged, ...
             info.positionError, ...
             info.orientationError, ...
             info.iterations);

    %--------------------------------------------------------------
    % Stop if IK fails
    %--------------------------------------------------------------

    if ~info.converged

        warning("IK failed at waypoint %d.",i);

        break;

    end

    %--------------------------------------------------------------
    % Use current solution as seed for next waypoint
    %--------------------------------------------------------------

    qSeed = qSolution;

end

%% Plot desired path vs FK path

figure;

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o-');

hold on;

plot3( ...
    fkPositions(1,:), ...
    fkPositions(2,:), ...
    fkPositions(3,:), ...
    'x--');

% Plot original user-defined points
plot3( ...
    P(1,:), ...
    P(2,:), ...
    P(3,:), ...
    's', ...
    'MarkerSize',8);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

legend( ...
    'Generated Cartesian path', ...
    'FK from IK solutions', ...
    'Input waypoints');

title('Cartesian Path: Desired vs IK + FK');

%% Display overall test results

successfulWaypoints = sum(converged);

fprintf('\n========================================\n');
fprintf('IK TEST RESULTS\n');
fprintf('========================================\n');
fprintf('Input waypoints:       %d\n',numInputPoints);
fprintf('Generated waypoints:   %d\n',numWaypoints);
fprintf('Successful waypoints:  %d\n',successfulWaypoints);
fprintf('Success rate:          %.1f %%\n', ...
    100*successfulWaypoints/numWaypoints);

if successfulWaypoints > 0

    fprintf('Maximum position error: %.3e m\n', ...
        max(positionErrors(1:successfulWaypoints)));

    fprintf('Maximum orientation error: %.3e rad\n', ...
        max(orientationErrors(1:successfulWaypoints)));

end

fprintf('========================================\n');

