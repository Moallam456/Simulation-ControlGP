%% ADLS Sigma Threshold Sweep
%
% PURPOSE
% Sweep different values of the ADLS singularity threshold c while keeping
% every other part of the experiment fixed.
%
% We keep constant:
%
%   - Same robot
%   - Same Cartesian path
%   - Same number of waypoints
%   - Same starting configuration
%   - Same constant TCP orientation
%   - Same sequential IK seeding
%   - Same lambdaMax
%   - Same convergence tolerances inside ADLS_IK
%
% The ONLY variable changed is:
%
%       sigmaThreshold = c
%
% ADLS damping law:
%
%   lambda = 0                              if sigmaMin > c
%
%   lambda = lambdaMax *
%            sqrt(1 - (sigmaMin/c)^2)       if sigmaMin <= c
%
% The goal is to determine whether Yang's c = 0.06 is appropriately
% scaled for our robot model.

clear;
clc;
close all;

%% -------------------------------------------------------------
% Robot
% --------------------------------------------------------------

robot = config.UR5();

%% -------------------------------------------------------------
% Same Cartesian path used in previous sequential ADLS test
% --------------------------------------------------------------

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

%% -------------------------------------------------------------
% Starting configuration
% --------------------------------------------------------------

qStart = zeros(robot.dof,1);

%% -------------------------------------------------------------
% Constant TCP orientation
% --------------------------------------------------------------

TStart = controlFK(robot,qStart);

RStart = TStart(1:3,1:3);
REnd   = RStart;

%% -------------------------------------------------------------
% Generate Cartesian pose path ONCE
%
% The exact same TPath is reused for every value of c.
% --------------------------------------------------------------

[TPath, sValues] = generateCartesianPosePath( ...
    pStart, ...
    pEnd, ...
    RStart, ...
    REnd, ...
    numWaypoints);

%% -------------------------------------------------------------
% ADLS parameters
% --------------------------------------------------------------

% Keep lambdaMax fixed during this experiment.
%
% We are NOT tuning lambdaMax yet because we want to isolate the effect
% of sigmaThreshold only.

lambdaMax = 0.1;

%% -------------------------------------------------------------
% Sigma threshold values to test
%
% Current observed sigmaMin values along our path were roughly:
%
%       1e-6 to 1e-4
%
% Yang's:
%
%       c = 0.06 = 6e-2
%
% is many orders of magnitude larger.
%
% Therefore we sweep logarithmically around the scale actually observed
% in our robot Jacobian.
% --------------------------------------------------------------

sigmaThresholds = [
    1e-6
    3e-6
    1e-5
    3e-5
    1e-4
    3e-4
    1e-3
    3e-3
    1e-2
    3e-2
    6e-2
];

numberOfTests = numel(sigmaThresholds);

%% -------------------------------------------------------------
% Storage
% --------------------------------------------------------------

SuccessfulWaypoints = zeros(numberOfTests,1);

FirstFailedWaypoint = nan(numberOfTests,1);

TotalIterations = zeros(numberOfTests,1);

DampingActivatedWaypoints = zeros(numberOfTests,1);

MaxLambdaUsed = zeros(numberOfTests,1);

MinSigmaEncountered = nan(numberOfTests,1);

MaxPositionError_m = nan(numberOfTests,1);

MaxOrientationError_rad = nan(numberOfTests,1);

%% =============================================================
% Sweep sigmaThreshold
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('             ADLS SIGMA THRESHOLD SWEEP\n');
fprintf('============================================================\n\n');

for testIndex = 1:numberOfTests

    sigmaThreshold = sigmaThresholds(testIndex);

    fprintf('------------------------------------------------------------\n');
    fprintf('TEST %d / %d\n',testIndex,numberOfTests);
    fprintf('sigmaThreshold c = %.3e\n',sigmaThreshold);
    fprintf('lambdaMax        = %.3e\n\n',lambdaMax);

    %% ---------------------------------------------------------
    % Reset solver state for this threshold
    %
    % Every threshold begins from the SAME initial configuration.
    % ----------------------------------------------------------

    qSeed = qStart;

    successful = 0;

    totalIterations = 0;

    dampingCount = 0;

    allLambda = [];
    allSigma = [];

    positionErrors = [];
    orientationErrors = [];

    %% ---------------------------------------------------------
    % Sequential Cartesian path IK
    % ----------------------------------------------------------

    for waypointIndex = 1:numWaypoints

        TTarget = TPath(:,:,waypointIndex);

        %% Run ADLS with selected sigma threshold

        [qSolution,info] = ADLS_IK( ...
            robot, ...
            TTarget, ...
            qSeed, ...
            lambdaMax, ...
            sigmaThreshold);

        totalIterations = ...
            totalIterations + info.iterations;

        %% -----------------------------------------------------
        % Store ADLS internal diagnostics
        % ------------------------------------------------------

        if isfield(info,'lambdaHistory') && ...
           ~isempty(info.lambdaHistory)

            validLambda = ...
                info.lambdaHistory( ...
                isfinite(info.lambdaHistory));

            allLambda = [
                allLambda
                validLambda(:)
            ];

            if any(validLambda > 0)
                dampingCount = dampingCount + 1;
            end

        end

        if isfield(info,'sigmaMinHistory') && ...
           ~isempty(info.sigmaMinHistory)

            validSigma = ...
                info.sigmaMinHistory( ...
                isfinite(info.sigmaMinHistory));

            allSigma = [
                allSigma
                validSigma(:)
            ];

        end

        %% -----------------------------------------------------
        % Independent FK pose evaluation
        %
        % We evaluate the returned solution ourselves rather than relying
        % only on the solver's internal error values.
        % ------------------------------------------------------

        TActual = controlFK(robot,qSolution);

        positionError = norm( ...
            TTarget(1:3,4) - ...
            TActual(1:3,4));

        orientationError = rotationError( ...
            TTarget(1:3,1:3), ...
            TActual(1:3,1:3));

        positionErrors(end+1) = positionError;

        orientationErrors(end+1) = orientationError;

        %% -----------------------------------------------------
        % Console information
        % ------------------------------------------------------

        fprintf( ...
            ['Waypoint %2d | conv %d | iter %4d | ' ...
             'pos %.2e | ori %.2e | lambda %.2e | sigma %.2e\n'], ...
            waypointIndex, ...
            info.converged, ...
            info.iterations, ...
            positionError, ...
            orientationError, ...
            info.lambda, ...
            info.sigmaMin);

        %% -----------------------------------------------------
        % Stop path if this threshold fails
        % ------------------------------------------------------

        if ~info.converged

            FirstFailedWaypoint(testIndex) = waypointIndex;

            fprintf( ...
                '\nFAILED at waypoint %d for c = %.3e\n\n', ...
                waypointIndex, ...
                sigmaThreshold);

            break;

        end

        %% -----------------------------------------------------
        % Successful waypoint
        % ------------------------------------------------------

        successful = successful + 1;

        % Sequential seeding:
        %
        % previous IK solution becomes the seed for the next Cartesian pose.

        qSeed = qSolution;

    end

    %% ---------------------------------------------------------
    % Save results for this threshold
    % ----------------------------------------------------------

    SuccessfulWaypoints(testIndex) = successful;

    TotalIterations(testIndex) = totalIterations;

    DampingActivatedWaypoints(testIndex) = dampingCount;

    if ~isempty(allLambda)

        MaxLambdaUsed(testIndex) = ...
            max(allLambda);

    end

    if ~isempty(allSigma)

        MinSigmaEncountered(testIndex) = ...
            min(allSigma);

    end

    if ~isempty(positionErrors)

        MaxPositionError_m(testIndex) = ...
            max(positionErrors);

    end

    if ~isempty(orientationErrors)

        MaxOrientationError_rad(testIndex) = ...
            max(orientationErrors);

    end

end

%% =============================================================
% Results table
% =============================================================

resultsTable = table( ...
    sigmaThresholds, ...
    SuccessfulWaypoints, ...
    FirstFailedWaypoint, ...
    TotalIterations, ...
    DampingActivatedWaypoints, ...
    MaxLambdaUsed, ...
    MinSigmaEncountered, ...
    MaxPositionError_m, ...
    MaxOrientationError_rad, ...
    'VariableNames', { ...
        'SigmaThreshold', ...
        'SuccessfulWaypoints', ...
        'FirstFailedWaypoint', ...
        'TotalIterations', ...
        'DampingActivatedWaypoints', ...
        'MaxLambdaUsed', ...
        'MinSigmaEncountered', ...
        'MaxPositionError_m', ...
        'MaxOrientationError_rad'});

fprintf('\n');
fprintf('============================================================\n');
fprintf('               SIGMA THRESHOLD RESULTS\n');
fprintf('============================================================\n');

disp(resultsTable);

%% =============================================================
% Find best threshold by path completion
% =============================================================
%
% First criterion:
%
%       maximize number of successfully completed waypoints
%
% Second criterion:
%
%       among equally successful thresholds, choose the one requiring
%       fewer total IK iterations.
%
% This does NOT yet mean this is the final industrial tuning.
% It only identifies promising values for further testing.

maxSuccess = max(SuccessfulWaypoints);

candidateIndices = ...
    find(SuccessfulWaypoints == maxSuccess);

[~,bestRelativeIndex] = ...
    min(TotalIterations(candidateIndices));

bestIndex = ...
    candidateIndices(bestRelativeIndex);

fprintf('\n');
fprintf('============================================================\n');
fprintf('                 BEST RESULT IN THIS SWEEP\n');
fprintf('============================================================\n');

fprintf( ...
    'Best sigmaThreshold: %.3e\n', ...
    sigmaThresholds(bestIndex));

fprintf( ...
    'Successful waypoints: %d / %d\n', ...
    SuccessfulWaypoints(bestIndex), ...
    numWaypoints);

fprintf( ...
    'Total iterations: %d\n', ...
    TotalIterations(bestIndex));

fprintf( ...
    'Damping-active waypoints: %d\n', ...
    DampingActivatedWaypoints(bestIndex));

fprintf( ...
    'Maximum lambda used: %.3e\n', ...
    MaxLambdaUsed(bestIndex));

fprintf( ...
    'Minimum sigma encountered: %.3e\n', ...
    MinSigmaEncountered(bestIndex));

fprintf('============================================================\n');

%% =============================================================
% Plot 1: Successful waypoints vs sigma threshold
% =============================================================

figure;

semilogx( ...
    sigmaThresholds, ...
    SuccessfulWaypoints, ...
    'o-', ...
    'LineWidth',1.5);

grid on;

xlabel('Sigma Threshold c');

ylabel('Successful Waypoints');

title( ...
    'ADLS Path Completion vs Sigma Threshold');

yline( ...
    numWaypoints, ...
    '--', ...
    'Full Path');

%% =============================================================
% Plot 2: Total iterations vs sigma threshold
% =============================================================

figure;

semilogx( ...
    sigmaThresholds, ...
    TotalIterations, ...
    'o-', ...
    'LineWidth',1.5);

grid on;

xlabel('Sigma Threshold c');

ylabel('Total ADLS Iterations');

title( ...
    'ADLS Computational Effort vs Sigma Threshold');

%% =============================================================
% Plot 3: Damping activation vs sigma threshold
% =============================================================

figure;

semilogx( ...
    sigmaThresholds, ...
    DampingActivatedWaypoints, ...
    'o-', ...
    'LineWidth',1.5);

grid on;

xlabel('Sigma Threshold c');

ylabel('Waypoints Where Damping Activated');

title( ...
    'ADLS Damping Activation vs Sigma Threshold');


%% ========================================================================
% Local function: true rotation-angle error
% ========================================================================

function angle = rotationError(RTarget,RActual)

RRelative = ...
    RTarget * RActual';

c = ...
    (trace(RRelative) - 1) / 2;

% Numerical protection for acos.

c = max(-1,min(1,c));

angle = acos(c);

end