%% Cartesian Pose Path + Sequential ADLS IK Test

clear;
clc;
close all;

robot = config.UR5();

%% Start and end Cartesian positions

pStart = [
    -0.8173
    -0.1915
    -0.088
];

pEnd = [
    -0.7173
    -0.0915
    -0.0055
];

numWaypoints = 50;

%% Starting robot configuration

qStart = zeros(robot.dof,1);

%% Keep TCP orientation constant

TStart = controlFK(robot,qStart);

RStart = TStart(1:3,1:3);
REnd   = RStart;

%% Generate full Cartesian pose path

[TPath, sValues] = generateCartesianPosePath( ...
    pStart, ...
    pEnd, ...
    RStart, ...
    REnd, ...
    numWaypoints);

%% Allocate joint-path storage

qPath = zeros(robot.dof,numWaypoints);

converged = false(1,numWaypoints);

iterations = zeros(1,numWaypoints);

%% Singularity diagnostic storage
%
% sigmaMinReturned stores the smallest singular value of the Jacobian
% evaluated at the joint configuration returned by IK for each waypoint.
%
% Interpretation:
%
%   sigmaMin large      -> configuration is farther from singularity
%
%   sigmaMin decreasing -> robot is approaching a singular configuration
%
%   sigmaMin -> 0       -> Jacobian loses rank and the robot becomes
%                          kinematically singular
%
% We calculate this directly from:
%
%       J(qSolution)
%
% rather than using the minimum value encountered during all ADLS
% iterations. This tells us how singular the actual returned robot
% configuration is at each Cartesian waypoint.

sigmaMinReturned = nan(1,numWaypoints);

%% Store every IK-returned configuration
%
% Unlike qPath, which contains only successful trajectory points,
% qReturned also stores the configuration returned by a FAILED solve.
%
% This lets us inspect waypoint 4 even though ADLS did not converge.

qReturned = nan(robot.dof,numWaypoints);

%% Initial IK seed
%
% First pose uses the current robot configuration.
% Every following pose uses the previous successful IK solution.

qSeed = qStart;

%% Track final attempted waypoint

lastAttemptedWaypoint = 0;

%% Solve IK sequentially

for i = 1:numWaypoints

    lastAttemptedWaypoint = i;

    TTarget = TPath(:,:,i);

    %% Run ADLS IK

    [qSolution, info] = ADLS_IK( ...
        robot, ...
        TTarget, ...
        qSeed);

    converged(i) = info.converged;
    iterations(i) = info.iterations;

    %% Store returned configuration even if IK fails

    qReturned(:,i) = qSolution;

    %% ---------------------------------------------------------
    % Singularity proximity at returned configuration
    % ----------------------------------------------------------
    %
    % Perform SVD:
    %
    %       J = U * Sigma * V'
    %
    % The smallest singular value of J is commonly used as an
    % indicator of proximity to a kinematic singularity.

    JReturned = controlJacobian(robot,qSolution);

    singularValues = svd(JReturned);

    sigmaMinReturned(i) = min(singularValues);

    %% Console output

    fprintf( ...
        ['Waypoint %2d/%2d | Converged: %d | Iterations: %d | ' ...
         'sigmaMin: %.3e | q5: %.3f deg\n'], ...
        i, ...
        numWaypoints, ...
        info.converged, ...
        info.iterations, ...
        sigmaMinReturned(i), ...
        rad2deg(qSolution(5)));

    %% Stop if IK fails

    if ~info.converged

        fprintf('\nFAILED WAYPOINT DIAGNOSTICS\n');

        fprintf('Waypoint: %d\n',i);

        fprintf( ...
            'Position error: %.6e m\n', ...
            info.positionError);

        fprintf( ...
            'Orientation error: %.6e rad\n', ...
            info.orientationError);

        fprintf( ...
            'Final sigmaMin reported by ADLS: %.6e\n', ...
            info.sigmaMin);

        fprintf( ...
            'sigmaMin at returned q: %.6e\n', ...
            sigmaMinReturned(i));

        fprintf( ...
            'Final lambda: %.6e\n', ...
            info.lambda);

        fprintf( ...
            'Final RMS dq: %.6e rad\n', ...
            info.rmsJointUpdate);

        fprintf('Seed used [deg]:\n');

        disp(rad2deg(qSeed).');

        fprintf('Returned q [deg]:\n');

        disp(rad2deg(qSolution).');

        warning( ...
            'ADLS IK failed at waypoint %d.', ...
            i);

        break;

    end

    %% Store successful solution

    qPath(:,i) = qSolution;

    %% Previous solution becomes next seed

    qSeed = qSolution;

end

%% Results

successfulWaypoints = sum(converged);

fprintf('\n========================================\n');
fprintf('SEQUENTIAL ADLS TEST RESULTS\n');
fprintf('========================================\n');

fprintf( ...
    'Total waypoints:      %d\n', ...
    numWaypoints);

fprintf( ...
    'Successful waypoints: %d\n', ...
    successfulWaypoints);

fprintf( ...
    'Success rate:         %.1f %%\n', ...
    100*successfulWaypoints/numWaypoints);

fprintf('========================================\n');

%% Plot successful joint path

if successfulWaypoints > 0

    figure;

    plot( ...
        1:successfulWaypoints, ...
        rad2deg( ...
            qPath(:,1:successfulWaypoints)).');

    grid on;

    xlabel('Waypoint');
    ylabel('Joint Angle [deg]');

    legend( ...
        'q_1','q_2','q_3','q_4','q_5','q_6', ...
        'Location','best');

    title('Sequential ADLS IK Joint Path');

end

%% =============================================================
% Singularity Diagnostic 1:
% sigmaMin versus waypoint
% =============================================================
%
% This plot answers:
%
%   "Is the Cartesian path driving the robot toward a singularity?"
%
% If sigmaMin falls toward zero as the waypoint number increases,
% the path is approaching a singular robot configuration.
%
% The horizontal line at c = 0.06 is the current ADLS damping
% activation threshold copied from Yang et al.
%
% IMPORTANT:
% c = 0.06 has NOT yet been tuned specifically for our robot.

figure;

semilogy( ...
    1:lastAttemptedWaypoint, ...
    sigmaMinReturned(1:lastAttemptedWaypoint), ...
    'o-', ...
    'LineWidth',1.5);

hold on;

yline( ...
    0.06, ...
    '--', ...
    'ADLS threshold c = 0.06');

grid on;

xlabel('Waypoint');

ylabel('\sigma_{min}(J)');

title( ...
    'Jacobian Minimum Singular Value Along Cartesian Path');

%% =============================================================
% Singularity Diagnostic 2:
% q5 versus waypoint
% =============================================================
%
% For a UR5-type spherical wrist, q5 approaching 0 deg is associated
% with the wrist-singularity region.
%
% Therefore this plot lets us compare:
%
%       q5 -> 0 deg
%
% with:
%
%       sigmaMin -> 0
%
% If both happen together, that strongly supports the conclusion
% that the trajectory is approaching a wrist singularity.

figure;

plot( ...
    1:lastAttemptedWaypoint, ...
    rad2deg( ...
        qReturned(5,1:lastAttemptedWaypoint)), ...
    'o-', ...
    'LineWidth',1.5);

hold on;

yline( ...
    0, ...
    '--', ...
    'q_5 = 0 deg');

grid on;

xlabel('Waypoint');

ylabel('q_5 [deg]');

title( ...
    'Wrist Joint q_5 Along Cartesian Path');

%% =============================================================
% Print joint configurations for attempted waypoints
% =============================================================
%
% This table allows us to inspect all six joints rather than only q5.
%
% We are particularly interested in whether q5 moves toward zero
% while sigmaMin simultaneously becomes very small.

qReturnedDeg = ...
    rad2deg( ...
        qReturned(:,1:lastAttemptedWaypoint)).';

diagnosticTable = table( ...
    (1:lastAttemptedWaypoint).', ...
    converged(1:lastAttemptedWaypoint).', ...
    sigmaMinReturned(1:lastAttemptedWaypoint).', ...
    qReturnedDeg(:,1), ...
    qReturnedDeg(:,2), ...
    qReturnedDeg(:,3), ...
    qReturnedDeg(:,4), ...
    qReturnedDeg(:,5), ...
    qReturnedDeg(:,6), ...
    'VariableNames', { ...
        'Waypoint', ...
        'Converged', ...
        'SigmaMin', ...
        'q1_deg', ...
        'q2_deg', ...
        'q3_deg', ...
        'q4_deg', ...
        'q5_deg', ...
        'q6_deg'});

fprintf('\n');
fprintf('========================================\n');
fprintf('SINGULARITY PATH DIAGNOSTICS\n');
fprintf('========================================\n');

disp(diagnosticTable);